"""CPU functionality."""

import sys


class CPU:
    """Main CPU class."""

    def __init__(self):
        """Construct a new CPU."""
        self.ram = [0] * 256
        self.reg = [0] * 7 + [0xF4]
        self.pc = 0

    def load(self):
        """Load a program into memory."""

        # Extract filename from command line
        try:
            filename = sys.argv[1]
            print(filename)
        except IndexError:
            print("Usage: python3 ls8.py <program_name>")
            sys.exit(1)

        #Validate filetype and confirm file exists
        if filename[-4:] != '.ls8':
            print("You must supply a '.ls8' binary.")
            sys.exit(2)
        try:
            f = open(filename)
        except FileNotFoundError:
            print(f"Invalid filename: {filename}")
            sys.exit(3)
        
        # Read the contents of the file
        address = 0
        for line in f:
            try:
                opcode = line.split()[0]
            except IndexError:
                continue
            if opcode == '#':
                continue
            self.ram[address] = int(opcode,2)
            address += 1
        f.close()

    def ram_read(self, addr):
        return self.ram[addr]

    def ram_write(self, addr, val):
        self.ram[addr] = val

    def alu(self, op, reg_a, reg_b):
        """ALU operations."""

        if op == "ADD":
            self.reg[reg_a] += self.reg[reg_b]
        # elif op == "SUB": etc
        else:
            raise Exception("Unsupported ALU operation")

    def trace(self):
        """
        Handy function to print out the CPU state. You might want to call this
        from run() if you need help debugging.
        """

        print(f"TRACE: %02X | %02X %02X %02X |" % (
            self.pc,
            # self.fl,
            # self.ie,
            self.ram_read(self.pc),
            self.ram_read(self.pc + 1),
            self.ram_read(self.pc + 2)
        ), end='')

        for i in range(8):
            print(" %02X" % self.reg[i], end='')

        print()

    def run(self):
        """Run the CPU."""

        # Microcode
        def HLT():
            return True

        def JMP():
            self.pc = self.ram[self.pc+1]

        def LDI():
            addr = self.ram[self.pc + 1]
            val = self.ram[self.pc + 2]
            self.reg[addr] = val

        def PRN():
            addr = self.ram[self.pc + 1]
            val = self.reg[addr]
            print(val)

        # Instruction mapping
        instructions = {
            0x01: HLT,
            0x82: LDI,
            0x47: PRN,
            0x54: JMP,
        }
        # Execution loop
        while True:
            ir = self.ram[self.pc]
            if ir not in instructions:
                print(f"Invalid opcode: {hex(ir)}")
                sys.exit(4)
            # If this is an ALU operation
            if (ir >> 5) & 1:
                pass
            # Execute non-ALU op - only HLT returns True
            elif instructions[ir]():
                break
            # If the instruction doesn't set the PC
            if not (ir >> 4) & 1:
                # Increment the PC based on the number of operands
                self.pc += (ir >> 6) + 1
