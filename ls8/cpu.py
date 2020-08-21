"""CPU functionality."""

import sys
from time import time

# Reserved registers
IM = 5
IS = 6
SP = 7


class CPU:
    """Main CPU class."""

    def __init__(self):
        """Construct a new CPU."""
        self.ram = [0] * 256  # RAM
        self.reg = [0] * 8    # General-purpose registers
        self.reg[SP] = 0xF4   # Stack Pointer
        self.pc = 0           # Program Counter
        self.fl = 0           # Flags Register

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

    def alu(self, op, regA, regB):
        """ALU operations."""

        # Microcode
        def ADD():
            self.reg[regA] += self.reg[regB]

        def AND():
            self.reg[regA] &= self.reg[regB]

        def CMP():
            a, b = self.reg[regA], self.reg[regB]
            cmp_bits = 0b100 if a < b else 0b010 if a > b else 0b001
            # Set CMP bits without altering others
            self.fl = (self.fl & 0b11111000) | cmp_bits

        def DEC():
            self.reg[regA] -= 1

        def DIV():
            self.reg[regA] //= self.reg[regB]

        def INC():
            self.reg[regA] += 1

        def MOD():
            if self.reg[regB] == 0:
                self.fl |= 0x80  # Set HLT flag
            else:
                self.reg[regA] %= self.reg[regB]

        def MUL():
            self.reg[regA] *= self.reg[regB]

        def NOT():
            self.reg[regA] = ~self.reg[regA]

        def OR():
            self.reg[regA] |= self.reg[regB]

        def SHL():
            self.reg[regA] <<= self.reg[regB]

        def SHR():
            self.reg[regA] >>= self.reg[regB]

        def SUB():
            self.reg[regA] -= self.reg[regB]

        def XOR():
            self.reg[regA] ^= self.reg[regB]

        # Instruction mapping
        instructions = {
            0xA0: ADD,
            0xA8: AND,
            0xA7: CMP,
            0x66: DEC,
            0xA3: DIV,
            0x65: INC,
            0xA4: MOD,
            0xA2: MUL,
            0x69: NOT,
            0xAA: OR,
            0xAC: SHL,
            0xAD: SHR,
            0xA1: SUB,
            0xAB: XOR,
        }

        try:
            # Perform the operation
            instructions[op]()
            # Truncate the A register to 8 bits
            self.reg[regA] &= 0xFF

        except KeyError:
            print(f"Invalid ALU operation: {hex(op)}")
            self.fl |= 0x80  # Set HLT flag

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

        def CALL():
            PUSH(self.pc+2)
            self.pc = self.reg[self.ram[self.pc+1]]

        def HLT():
            self.fl |= 0x80  # set HLT flag

        def INT():
            n = self.reg[self.ram[self.pc+1]]
            self.reg[IS] |= 1 << n

        def Interrupt(n):
            # Clear the given interrupt
            mask = 0xFF ^ (1 << n)
            self.reg[IS] &= mask
            # Push machine state onto the stack
            PUSH(self.pc)
            PUSH(self.fl)
            for i in range(7):
                PUSH(self.reg[i])
            # Set program counter to interrupt vector
            self.pc = self.ram[0xF8 + n]
            # Disable Interrupts
            self.reg[IM] = 0x00

        def IRET():
            # Pop machine state off of stack
            for i in range(7):
                self.reg[6-i] = POP(return_=True)
            self.fl = POP(True)
            self.pc = POP(True)

        def Jump(condition=True):
            if condition:
                self.pc = self.reg[self.ram[self.pc+1]]
            else:
                self.pc += 2

        def JEQ():  # LGE
            Jump(self.fl & 0b1)

        def JGE():
            Jump(not self.fl & 0b100)

        def JGT():
            Jump(self.fl & 0b10)

        def JLE():
            Jump(not self.fl & 0b10)

        def JLT():
            Jump(self.fl & 0b100)

        def JMP():
            Jump()

        def JNE():
            Jump(not self.fl & 0b1)

        def LD():
            reg = self.ram[self.pc+1]
            addr = self.reg[self.pc+2]
            val = self.ram[addr]
            self.reg[reg] = val

        def LDI():
            addr = self.ram[self.pc + 1]
            val = self.ram[self.pc + 2]
            self.reg[addr] = val

        def NOP():
            pass

        def POP(return_=False):
            val = self.ram[self.reg[SP]]
            # If called by the CPU, extract operand
            if not return_:
                reg = self.ram[self.pc+1]
                self.reg[reg] = val
            # Increment stack pointer
            self.reg[SP] = (self.reg[SP]+1) & 0xFF
            # If called by another function, return popped value
            if return_:
                return val

        def PRA():
            addr = self.ram[self.pc+1]
            val = self.reg[addr]
            print(chr(val))

        def PRN():
            addr = self.ram[self.pc + 1]
            val = self.reg[addr]
            print(val)

        def PUSH(val=None):
            # Decrement stack pointer
            self.reg[SP] = (self.reg[SP]-1) & 0xFF
            # If called by the CPU, extract operand
            if val is None:
                val = self.reg[self.ram[self.pc+1]]
            self.ram[self.reg[SP]] = val

        def RET():
            self.pc = POP(return_=True)

        def ST():
            regA = self.ram[self.pc+1]
            regB = self.ram[self.pc+2]
            self.ram[self.reg[regA]] = self.reg[regB]

        # Instruction mapping
        instructions = {
            0x50: CALL,
            0x01: HLT,
            0x52: INT,
            0x13: IRET,
            0x55: JEQ,
            0x5A: JGE,
            0x57: JGT,
            0x59: JLE,
            0x58: JLT,
            0x54: JMP,
            0x56: JNE,
            0x83: LD,
            0x82: LDI,
            0x00: NOP,
            0x46: POP,
            0x48: PRA,
            0x47: PRN,
            0x45: PUSH,
            0x11: RET,
            0x84: ST,
        }

        # Configure Interrupts
        last_interrupt = time()
        # Execution loop
        self.pc = 0
        while True:
            # Trigger time-based interrupt every second
            if time() - last_interrupt > 1:
                self.reg[IS] |= 1
                last_interrupt = time()
            # Trigger keyboard interrupt

            # Handle Interrupts
            maskedInterrupts = self.reg[IM] & self.reg[IS]
            if maskedInterrupts:
                for i in range(8):
                    if maskedInterrupts & 1:
                        Interrupt(i)
                        break
                    maskedInterrupts >>= 1

            ir = self.ram[self.pc]

            # Execute non-ALU op
            if ~(ir >> 5) & 1:
                try:
                    instructions[ir]()
                except KeyError:
                    print(f"Invalid operation at {self.pc}:{hex(ir)}")
                    self.fl |= 0x80  # Set HLT flag

            # Execute ALU op
            else:
                regA = self.ram[self.pc+1]
                regB = self.ram[self.pc+2] if (ir >> 6) > 1 else None
                self.alu(ir, regA, regB)

            # If the HLT flag was set
            if self.fl >> 7:
                break

            # If the instruction doesn't set the PC
            if ~(ir >> 4) & 1:
                # Increment it based on the number of operands
                self.pc += (ir >> 6) + 1


if __name__ == "__main__":
    cpu = CPU()

    cpu.load()
    cpu.run()
