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

        # Validate filetype and confirm file exists
        if filename[-4:] != '.ls8':
            print("You must supply a '.ls8' binary.")
            sys.exit(2)
        try:
            f = open(filename)
        except FileNotFoundError:
            print(f"File not found: {filename}")
            sys.exit(2)

        # Read the contents of the file
        address = 0
        for line in f:
            try:
                opcode = line.split()[0]
            except IndexError:
                continue
            if opcode == '#':
                continue
            self.ram[address] = int(opcode, 2)
            address += 1
        f.close()

        # Double-check the file wasn't empty
        if address == 0:
            print("Error: Empty source file")
            sys.exit(2)

    def ram_read(self, addr):
        return self.ram[addr]

    def ram_write(self, addr, val):
        self.ram[addr] = val

    def alu(self, op):
        """ALU operations."""

        #Microcode
        def MUL():
            regA = self.ram[self.pc+1]
            regB = self.ram[self.pc+2]
            self.reg[regA] *= self.reg[regB]

        # Instruction mapping
        instructions = {
            0x2: MUL
        }
        
        # Execution
        try:
            instructions[op & 0xF]()
        except KeyError:
            print(f"Unsupported ALU operation: {hex(op)}")
            sys.exit(4)

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
        def NOP():
            pass

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
            0x0: NOP,
            0x1: HLT,
            0x2: LDI,
            0x4: JMP,
             0x7: PRN,
        }

        # Execution loop
        self.pc = 0
        while True:
            ir = self.ram[self.pc]
            # If this is an ALU operation
            if (ir >> 5) & 1:
                self.alu(ir)
            # Validate non-ALU opcode
            elif ir & 0xF not in instructions:
                print(f"Invalid opcode: {hex(ir)}")
                sys.exit(4)
            # Execute non-ALU op - only HLT returns True
            elif instructions[ir & 0xF]():
                break
            # If the instruction doesn't set the PC
            if not (ir >> 4) & 1:
                # Increment the PC based on the number of operands
                self.pc += (ir >> 6) + 1
