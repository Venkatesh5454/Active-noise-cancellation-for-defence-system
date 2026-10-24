#!/usr/bin/env python3
# =============================================================================
# se_asm.py - assembler for the v2 serial engine (rtl/se/se_engine.v)
# -----------------------------------------------------------------------------
# Turns a ".se" text program into the 16-bit instruction words that the
# serial engine's state machines run (docs/SPEC.md section 5).
#
#   python3 tools/se_asm.py [-o sw/se_programs.h] [--outdir programs] a.se [b.se ...]
#
# For every source file it writes
#   <name>.hex  one 4-hex-digit word per line (for Verilog $readmemh)
#   <name>.lst  listing: address, code, source line
# and, with -o, ONE C header with all the programs:
#   static const uint16_t se_prog_<name>[] = { ... };
#   #define SE_PROG_<NAME>_ORIGIN / _LEN  (and _CLKDIV / _PINCTRL / _SHIFTCTRL)
# Nothing is written if any file has an error.  Without -o no header is
# written (so assembling one test file never overwrites the shared header;
# tools/build_programs.sh rebuilds it from all programs/*.se).
#
# Source syntax (one instruction per line, case-insensitive mnemonics):
#
#   label: OPCODE args [delay] side v      // comment   (or ; comment)
#
#   JMP  [cond,] target     cond: !x x-- !y y-- x!=y pin !osre
#   WAIT level, pin n       stall until in[n] == level
#   IN   src, n             src: pins x y null time crc isr osr   n = 1..32
#   OUT  dst, n             dst: pins x y null pindirs pc isr crc n = 1..32
#   PUSH [block|noblock]    PULL [block|noblock]      (default block)
#   SET  dst, v             dst: pins x y pindirs     v = 0..31
#   OD   pin n, 0|Z         0 = pull low, Z (or 1) = release
#   CRC  n | CRC reset      n = 1..32
#   NOP                     = JMP to the next address
#
# Directives:
#   .program name      program name (default: the file name)
#   .side_set 1        every instruction gives "side 0|1"; delay <= 15
#                      (.side_set 0 = no side-set, the default; delay <= 31)
#   .origin N          load address 0..31 (default 0).  JMP targets are
#                      absolute: origin + offset of the label.  A numeric JMP
#                      target is also an offset inside the program.
#   .clkdiv INT FRAC   only for the C header (SMs_CLKDIV value)
#   .pinctrl 0x...     only for the C header (SMs_PINCTRL value)
#   .shiftctrl 0x...   only for the C header (SMs_SHIFTCTRL value, its
#                      START_PC field [12:8] should equal the origin)
# Numbers can be decimal, 0x hex or 0b binary.
# Python 3 standard library only.
# =============================================================================
import argparse
import os
import re
import sys

OPCODES = {"jmp": 0, "wait": 1, "in": 2, "out": 3, "push": 4, "pull": 4,
           "set": 5, "od": 6, "crc": 7, "nop": 0}
JMP_COND = {"": 0, "!x": 1, "x--": 2, "!y": 3, "y--": 4, "x!=y": 5,
            "pin": 6, "!osre": 7}
IN_SRC = {"pins": 0, "x": 1, "y": 2, "null": 3, "time": 4, "crc": 5,
          "isr": 6, "osr": 7}
OUT_DST = {"pins": 0, "x": 1, "y": 2, "null": 3, "pindirs": 4, "pc": 5,
           "isr": 6, "crc": 7}
SET_DST = {"pins": 0, "x": 1, "y": 2, "pindirs": 4}
IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
PROG_WORDS = 32


class AsmError(Exception):
    """An error in one source line."""


def parse_num(tok, what="number"):
    """Decimal, 0x hex or 0b binary.  Raises AsmError."""
    t = tok.strip().lower()
    try:
        if t.startswith(("0x", "0b")):
            return int(t, 0)
        if t.isdigit():
            return int(t, 10)
    except ValueError:
        pass
    raise AsmError("bad %s '%s' (use decimal, 0x.. or 0b..)" % (what, tok.strip()))


def check_range(v, lo, hi, what):
    if v < lo or v > hi:
        raise AsmError("%s %d is out of range %d..%d" % (what, v, lo, hi))
    return v


def split_args(text):
    """Split 'a, b' into ['a', 'b'] (empty text gives [])."""
    text = text.strip()
    if not text:
        return []
    return [p.strip() for p in text.split(",")]


class Program:
    def __init__(self, path):
        self.path = path
        stem = os.path.splitext(os.path.basename(path))[0]
        self.name = re.sub(r"[^A-Za-z0-9_]", "_", stem)
        if not re.match(r"^[A-Za-z_]", self.name):
            self.name = "p_" + self.name
        self.origin = 0
        self.side_set = 0
        self.clkdiv = None
        self.pinctrl = None
        self.shiftctrl = None
        self.lines = []        # (line number, source text)
        self.instrs = []       # dicts: line, mnem, args, delay, side, offset
        self.labels = {}       # name -> offset
        self.words = []
        self.errors = []
        self.warnings = []
        self.line_addr = {}    # line number -> (address, word)

    # ---------- messages ----------
    def err(self, line, msg):
        self.errors.append("%s:%d: error: %s" % (self.path, line, msg))

    def warn(self, line, msg):
        self.warnings.append("%s:%d: warning: %s" % (self.path, line, msg))

    # ---------- pass 1: read lines, labels, directives ----------
    def parse(self):
        try:
            with open(self.path) as f:
                text = f.read()
        except OSError as e:
            self.errors.append("%s: error: cannot read file (%s)" % (self.path, e.strerror))
            return
        seen_program = False
        for n, raw in enumerate(text.splitlines(), 1):
            self.lines.append((n, raw.rstrip()))
            line = re.split(r"//|;", raw, maxsplit=1)[0].strip()
            if not line:
                continue
            try:
                # label(s) at the start of the line
                while True:
                    m = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)\s*:(.*)$", line)
                    if not m:
                        break
                    lab = m.group(1)
                    if lab.lower() in self.labels_lower():
                        raise AsmError("label '%s' is defined twice" % lab)
                    self.labels[lab] = len(self.instrs)
                    line = m.group(2).strip()
                if not line:
                    continue
                if line.startswith("."):
                    seen_program = self.directive(n, line, seen_program)
                    continue
                self.instrs.append(self.split_instr(n, line))
            except AsmError as e:
                self.err(n, str(e))

    def labels_lower(self):
        return {k.lower() for k in self.labels}

    def directive(self, n, line, seen_program):
        parts = line.replace(",", " ").split()
        d = parts[0].lower()
        args = parts[1:]
        before_code = (len(self.instrs) == 0)
        if d == ".program":
            if len(args) != 1 or not IDENT.match(args[0]):
                raise AsmError(".program needs one name made of letters, digits and _")
            if seen_program:
                raise AsmError(".program given twice")
            self.name = args[0]
            return True
        if d == ".side_set":
            if len(args) != 1:
                raise AsmError(".side_set takes one value: 1 (or 0); 'opt' is not supported")
            v = parse_num(args[0])
            if v not in (0, 1):
                raise AsmError(".side_set must be 0 or 1 (the engine has one side-set bit)")
            if not before_code:
                raise AsmError(".side_set must come before the first instruction")
            self.side_set = v
        elif d == ".origin":
            if len(args) != 1:
                raise AsmError(".origin takes one value")
            if not before_code:
                raise AsmError(".origin must come before the first instruction")
            self.origin = check_range(parse_num(args[0]), 0, PROG_WORDS - 1, "origin")
        elif d == ".clkdiv":
            if len(args) != 2:
                raise AsmError(".clkdiv needs INT and FRAC, e.g. .clkdiv 108 128")
            i = check_range(parse_num(args[0]), 0, 65535, "clkdiv INT")
            fr = check_range(parse_num(args[1]), 0, 255, "clkdiv FRAC")
            self.clkdiv = (i << 16) | (fr << 8)
        elif d == ".pinctrl":
            if len(args) != 1:
                raise AsmError(".pinctrl takes one value")
            v = check_range(parse_num(args[0]), 0, 0xFFFFFFFF, "pinctrl")
            if v & 0xF0008080:
                self.warn(n, "pinctrl sets unused bits (7, 15 or 31:28)")
            self.pinctrl = (v, n)
        elif d == ".shiftctrl":
            if len(args) != 1:
                raise AsmError(".shiftctrl takes one value")
            v = check_range(parse_num(args[0]), 0, 0xFFFFFFFF, "shiftctrl")
            if v & ~0x1F03 & 0xFFFFFFFF:
                self.warn(n, "shiftctrl sets unused bits (only 0, 1 and 12:8 exist)")
            self.shiftctrl = (v, n)
        else:
            raise AsmError("unknown directive '%s'" % parts[0])
        return seen_program

    def split_instr(self, n, line):
        """Pull out [delay] and 'side v', return the instruction dict."""
        delay = None
        side = None
        dm = re.findall(r"\[([^\]]*)\]", line)
        if len(dm) > 1:
            raise AsmError("more than one [delay]")
        if dm:
            delay = parse_num(dm[0], "delay")
            line = re.sub(r"\[[^\]]*\]", " ", line)
        if "[" in line or "]" in line:
            raise AsmError("unbalanced [ ] around the delay")
        sm = re.findall(r"\bside\b\s*(\S*)", line, flags=re.IGNORECASE)
        if len(sm) > 1:
            raise AsmError("more than one 'side'")
        if sm:
            if not sm[0]:
                raise AsmError("'side' needs a value 0 or 1")
            side = parse_num(sm[0], "side-set value")
            line = re.sub(r"\bside\b\s*\S*", " ", line, flags=re.IGNORECASE)
        parts = line.strip().split(None, 1)
        mnem = parts[0].lower()
        args = parts[1] if len(parts) > 1 else ""
        if mnem not in OPCODES:
            raise AsmError("unknown instruction '%s'" % parts[0])
        return {"line": n, "mnem": mnem, "args": args.strip(), "delay": delay,
                "side": side, "offset": len(self.instrs)}

    # ---------- pass 2: encode ----------
    def assemble(self):
        if not self.instrs:
            if self.errors:
                return
            self.errors.append("%s: error: the program has no instructions" % self.path)
            return
        if self.origin + len(self.instrs) > PROG_WORDS:
            self.errors.append("%s: error: program does not fit: origin %d + %d words > %d"
                               % (self.path, self.origin, len(self.instrs), PROG_WORDS))
            return
        for ins in self.instrs:
            try:
                w = self.encode(ins)
                self.words.append(w)
                self.line_addr[ins["line"]] = (self.origin + ins["offset"], w)
            except AsmError as e:
                self.err(ins["line"], str(e))
        # configuration directives must agree with the program
        if self.pinctrl is not None:
            v, n = self.pinctrl
            if ((v >> 12) & 1) != self.side_set:
                self.err(n, "pinctrl SIDE_EN (bit 12) = %d but .side_set is %d"
                         % ((v >> 12) & 1, self.side_set))
        if self.shiftctrl is not None:
            v, n = self.shiftctrl
            if ((v >> 8) & 0x1F) != self.origin:
                self.warn(n, "shiftctrl START_PC = %d but .origin is %d"
                          % ((v >> 8) & 0x1F, self.origin))

    def target(self, tok):
        """A JMP target: label or number (offset in the program) -> absolute."""
        t = tok.strip()
        if IDENT.match(t):
            for k, off in self.labels.items():
                if k.lower() == t.lower():
                    return self.origin + off
            raise AsmError("unknown label '%s'" % t)
        off = parse_num(t, "jump target")
        return check_range(self.origin + off, 0, PROG_WORDS - 1, "jump target (origin + offset)")

    def encode(self, ins):
        mnem = ins["mnem"]
        args = ins["args"]
        a = split_args(args)
        low = [x.lower() for x in a]
        op = OPCODES[mnem]
        if mnem == "nop":
            if a:
                raise AsmError("NOP takes no arguments")
            arg = (self.origin + ins["offset"] + 1) % PROG_WORDS
        elif mnem == "jmp":
            if len(a) == 1:
                cond, tgt = "", a[0]
            elif len(a) == 2:
                cond, tgt = re.sub(r"\s+", "", low[0]), a[1]
            else:
                raise AsmError("JMP needs [cond,] target")
            if cond not in JMP_COND or (len(a) == 2 and cond == ""):
                raise AsmError("unknown JMP condition '%s' (use !x x-- !y y-- x!=y pin !osre)" % a[0])
            arg = (JMP_COND[cond] << 5) | self.target(tgt)
        elif mnem == "wait":
            t = [x for x in re.split(r"[\s,]+", args.lower()) if x]
            if len(t) == 3 and t[1] == "pin":
                lvl, pin = t[0], t[2]
            elif len(t) == 2:
                lvl, pin = t[0], t[1]
            else:
                raise AsmError("WAIT needs 'level, pin n'")
            lv = check_range(parse_num(lvl, "level"), 0, 1, "WAIT level")
            pn = check_range(parse_num(pin, "pin"), 0, 3, "WAIT pin")
            arg = (lv << 7) | pn
        elif mnem in ("in", "out"):
            table = IN_SRC if mnem == "in" else OUT_DST
            if len(a) != 2:
                raise AsmError("%s needs '%s, bit count'" % (mnem.upper(),
                               "source" if mnem == "in" else "destination"))
            if low[0] not in table:
                raise AsmError("unknown %s %s '%s' (use %s)" % (
                    mnem.upper(), "source" if mnem == "in" else "destination",
                    a[0], " ".join(table)))
            cnt = check_range(parse_num(a[1], "bit count"), 1, 32, "bit count")
            arg = (table[low[0]] << 5) | (cnt & 0x1F)
        elif mnem in ("push", "pull"):
            t = [x for x in re.split(r"[\s,]+", args.lower()) if x]
            if t == [] or t == ["block"]:
                blk = 1
            elif t == ["noblock"]:
                blk = 0
            else:
                raise AsmError("%s takes 'block' or 'noblock'" % mnem.upper())
            arg = ((1 if mnem == "pull" else 0) << 7) | (blk << 6)
        elif mnem == "set":
            if len(a) != 2:
                raise AsmError("SET needs 'destination, value'")
            if low[0] not in SET_DST:
                raise AsmError("unknown SET destination '%s' (use pins x y pindirs)" % a[0])
            v = check_range(parse_num(a[1], "value"), 0, 31, "SET value")
            arg = (SET_DST[low[0]] << 5) | v
        elif mnem == "od":
            t = [x for x in re.split(r"[\s,]+", args.lower()) if x]
            if len(t) == 3 and t[0] == "pin":
                pin, v = t[1], t[2]
            elif len(t) == 2:
                pin, v = t[0], t[1]
            else:
                raise AsmError("OD needs 'pin n, 0|Z'")
            pn = check_range(parse_num(pin, "pin"), 0, 3, "OD pin")
            if v == "z" or v == "1":
                rel = 1
            elif v == "0":
                rel = 0
            else:
                raise AsmError("OD value must be 0 (pull low) or Z (release)")
            arg = (rel << 7) | pn
        else:  # crc
            t = [x for x in re.split(r"[\s,]+", args.lower()) if x]
            if t == ["reset"]:
                arg = 0x80
            elif len(t) == 1:
                arg = check_range(parse_num(t[0], "bit count"), 1, 32, "CRC bit count") & 0x1F
            else:
                raise AsmError("CRC needs a bit count 1..32 or 'reset'")
        # delay / side-set field
        delay = ins["delay"] if ins["delay"] is not None else 0
        if self.side_set:
            if ins["side"] is None:
                raise AsmError("missing 'side 0|1' (the program uses .side_set 1)")
            check_range(ins["side"], 0, 1, "side-set value")
            check_range(delay, 0, 15, "delay (with side-set)")
            field = (ins["side"] << 4) | delay
        else:
            if ins["side"] is not None:
                raise AsmError("'side' used but the program has no .side_set 1")
            check_range(delay, 0, 31, "delay")
            field = delay
        return (op << 13) | (field << 8) | arg

    # ---------- outputs ----------
    def hex_text(self):
        return "".join("%04x\n" % w for w in self.words)

    def lst_text(self):
        out = ["; se_asm.py listing of %s" % self.path,
               "; program %s, origin %d, %d words, side_set %d"
               % (self.name, self.origin, len(self.words), self.side_set),
               ";",
               "; addr = word index in the program memory (decimal), code = hex",
               ";",
               "; addr code  line  source"]
        for n, src in self.lines:
            if n in self.line_addr:
                ad, w = self.line_addr[n]
                out.append("  %02d   %04x %4d  %s" % (ad, w, n, src))
            else:
                out.append("            %4d  %s" % (n, src))
        return "\n".join(out) + "\n"


def header_text(progs, hpath):
    guard = re.sub(r"[^A-Za-z0-9]", "_", os.path.basename(hpath)).upper()
    out = ["/*",
           " * %s - serial engine programs, generated by tools/se_asm.py." % os.path.basename(hpath),
           " * Do not edit: change the .se files in programs/ and run tools/build_programs.sh.",
           " *",
           " * Load a program:  for (i = 0; i < SE_PROG_X_LEN; i++)",
           " *                      PROG[SE_PROG_X_ORIGIN + i] = se_prog_x[i];",
           " * then write SMs_CLKDIV / PINCTRL / SHIFTCTRL, RESTART and enable the SM.",
           " */",
           "#ifndef %s" % guard,
           "#define %s" % guard,
           "",
           "#include <stdint.h>",
           ""]
    for p in progs:
        up = p.name.upper()
        out.append("/* %s: %s (%d words at %d) */" % (p.name, p.path.replace(os.sep, "/"),
                                                       len(p.words), p.origin))
        out.append("static const uint16_t se_prog_%s[] = {" % p.name)
        for i in range(0, len(p.words), 8):
            out.append("    " + ", ".join("0x%04x" % w for w in p.words[i:i + 8]) + ",")
        out.append("};")
        out.append("#define SE_PROG_%s_ORIGIN    %d" % (up, p.origin))
        out.append("#define SE_PROG_%s_LEN       %d" % (up, len(p.words)))
        if p.clkdiv is not None:
            out.append("#define SE_PROG_%s_CLKDIV    0x%08xu" % (up, p.clkdiv))
        if p.pinctrl is not None:
            out.append("#define SE_PROG_%s_PINCTRL   0x%08xu" % (up, p.pinctrl[0]))
        if p.shiftctrl is not None:
            out.append("#define SE_PROG_%s_SHIFTCTRL 0x%08xu" % (up, p.shiftctrl[0]))
        out.append("")
    out.append("#endif /* %s */" % guard)
    return "\n".join(out) + "\n"


def line_of(msg):
    """Line number in 'file:line: ...' (0 for whole-file messages), for sorting."""
    m = re.search(r":(\d+): ", msg)
    return int(m.group(1)) if m else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Assembler for the v2 serial engine")
    ap.add_argument("files", nargs="+", help=".se source files")
    ap.add_argument("-o", "--header", help="write one C header with all programs")
    ap.add_argument("--outdir", help="directory for .hex / .lst (default: next to each source)")
    a = ap.parse_args(argv)

    progs = []
    ok = True
    names = {}
    for path in a.files:
        p = Program(path)
        p.parse()
        p.assemble()
        for w in p.warnings:
            print(w, file=sys.stderr)
        for e in sorted(p.errors, key=line_of):
            print(e, file=sys.stderr)
        if p.errors:
            ok = False
        elif p.name.lower() in names:
            print("%s: error: program name '%s' is also used by %s"
                  % (path, p.name, names[p.name.lower()]), file=sys.stderr)
            ok = False
        else:
            names[p.name.lower()] = path
        progs.append(p)
    if not ok:
        print("se_asm: errors found, nothing written", file=sys.stderr)
        return 1

    for p in progs:
        d = a.outdir if a.outdir else (os.path.dirname(p.path) or ".")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, p.name + ".hex"), "w") as f:
            f.write(p.hex_text())
        with open(os.path.join(d, p.name + ".lst"), "w") as f:
            f.write(p.lst_text())
        print("se_asm: %s -> %s.hex/.lst (%d words at %d)"
              % (p.path, os.path.join(d, p.name), len(p.words), p.origin))
    if a.header:
        hd = os.path.dirname(a.header)
        if hd:
            os.makedirs(hd, exist_ok=True)
        with open(a.header, "w") as f:
            f.write(header_text(progs, a.header))
        print("se_asm: wrote %s (%d program%s)" % (a.header, len(progs), "" if len(progs) == 1 else "s"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
