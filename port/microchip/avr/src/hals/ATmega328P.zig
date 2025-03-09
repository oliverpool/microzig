const std = @import("std");
const micro = @import("microzig");
const peripherals = micro.chip.peripherals;
const USART0 = peripherals.USART0;

pub const cpu = micro.cpu;
const Port = enum(u8) {
    B = 1,
    C = 2,
    D = 3,
};

pub const clock = struct {
    pub const Domain = enum {
        cpu,
    };
};

pub fn parse_pin(comptime spec: []const u8) type {
    const invalid_format_msg = "The given pin '" ++ spec ++ "' has an invalid format. Pins must follow the format \"P{Port}{Pin}\" scheme.";

    if (spec.len != 3)
        @compileError(invalid_format_msg);
    if (spec[0] != 'P')
        @compileError(invalid_format_msg);

    const io = @field(micro.chip.peripherals, "PORT" ++ spec[1..2]);
    return struct {
        pub const index: u3 = std.fmt.parseInt(u3, spec[2..3], 10) catch @compileError(invalid_format_msg);

        pub const port = &@field(io, "PORT" ++ spec[1..2]);
        pub const pin = &@field(io, "PIN" ++ spec[1..2]);
        pub const ddr = &@field(io, "DDR" ++ spec[1..2]);

        pub const legacy = struct {
            pub const port: Port = std.meta.stringToEnum(Port, spec[1..2]) orelse @compileError(invalid_format_msg);
            pub const pin: u3 = std.fmt.parseInt(u3, spec[2..3], 10) catch @compileError(invalid_format_msg);
        };
    };
}

fn sfr_addr(comptime reg: *volatile u8) u5 {
    return @intFromPtr(reg) - 0x20;
}

fn assert5(comptime legacy: u5, comptime updated: u5) void {
    if (legacy != updated) {
        @compileLog("legacy", legacy, "updated", updated);
        @compileError("not equal");
    }
}

pub const gpio = struct {
    fn regs(comptime desc: type) type {
        return struct {
            // io address
            const pin_addr: u5 = 3 * @intFromEnum(desc.port) + 0x00;
            const dir_addr: u5 = 3 * @intFromEnum(desc.port) + 0x01;
            const port_addr: u5 = 3 * @intFromEnum(desc.port) + 0x02;

            // ram mapping
            const pin = @as(*volatile u8, @ptrFromInt(0x20 + @as(usize, pin_addr)));
            const dir = @as(*volatile u8, @ptrFromInt(0x20 + @as(usize, dir_addr)));
            const port = @as(*volatile u8, @ptrFromInt(0x20 + @as(usize, port_addr)));
        };
    }

    pub fn setOutput(comptime pin: type) void {
        assert5(regs(pin.legacy).dir_addr, sfr_addr(pin.ddr));
        comptime unreachable;
        cpu.sbi(sfr_addr(pin.ddr), pin.index);
    }

    pub fn setInput(comptime pin: type) void {
        assert5(regs(pin.legacy).dir_addr, sfr_addr(pin.ddr));
        comptime unreachable;
        cpu.cbi(sfr_addr(pin.ddr), pin.index);
    }

    pub fn read(comptime pin: type) micro.core.experimental.gpio.State {
        comptime std.debug.assert(regs(pin.legacy).pin == pin.pin);
        comptime unreachable;
        return if ((pin.pin.* & (1 << pin.index)) != 0)
            .high
        else
            .low;
    }

    pub fn write(comptime pin: type, state: micro.core.experimental.gpio.State) void {
        assert5(regs(pin.legacy).port_addr, sfr_addr(pin.port));
        comptime unreachable;
        switch (state) {
            .high => cpu.sbi(sfr_addr(pin.port), pin.index),
            .low => cpu.cbi(sfr_addr(pin.port), pin.index),
        }
    }

    pub fn toggle(comptime pin: type) void {
        assert5(regs(pin.legacy).pin_addr, sfr_addr(pin.pin));
        comptime unreachable;
        cpu.sbi(sfr_addr(pin.pin), pin.index);
    }
};

pub const uart = struct {
    pub const DataBits = enum {
        five,
        six,
        seven,
        eight,
        nine,
    };

    pub const StopBits = enum {
        one,
        two,
    };

    pub const Parity = enum {
        odd,
        even,
    };
};

pub fn Uart(comptime index: usize, comptime pins: micro.uart.Pins) type {
    if (index != 0) @compileError("Atmega328p only has a single uart!");
    if (pins.tx != null or pins.rx != null)
        @compileError("Atmega328p has fixed pins for uart!");

    return struct {
        const Self = @This();

        fn computeDivider(baud_rate: u32) !u12 {
            const pclk = micro.clock.get().cpu;
            const divider = ((pclk + (8 * baud_rate)) / (16 * baud_rate)) - 1;

            return std.math.cast(u12, divider) orelse return error.UnsupportedBaudRate;
        }

        fn computeBaudRate(divider: u12) u32 {
            return micro.clock.get().cpu / (16 * @as(u32, divider) + 1);
        }

        pub fn init(config: micro.uart.Config) !Self {
            const ucsz: u3 = switch (config.data_bits) {
                .five => 0b000,
                .six => 0b001,
                .seven => 0b010,
                .eight => 0b011,
                .nine => return error.UnsupportedWordSize, // 0b111
            };

            const upm: u2 = if (config.parity) |parity| switch (parity) {
                .even => @as(u2, 0b10), // even
                .odd => @as(u2, 0b11), // odd
            } else 0b00; // parity disabled

            const usbs: u1 = switch (config.stop_bits) {
                .one => 0b0,
                .two => 0b1,
            };

            const umsel: u2 = 0b00; // Asynchronous USART

            // baud is computed like this:
            //             f(osc)
            // BAUD = ----------------
            //        16 * (UBRRn + 1)

            const ubrr_val = try computeDivider(config.baud_rate);

            USART0.UCSR0A.modify(.{
                .MPCM0 = 0,
                .U2X0 = 0,
            });
            USART0.UCSR0B.write(.{
                .TXB80 = 0, // we don't care about these btw
                .RXB80 = 0, // we don't care about these btw
                .UCSZ02 = @as(u1, @truncate((ucsz & 0x04) >> 2)),
                .TXEN0 = 1,
                .RXEN0 = 1,
                .UDRIE0 = 0, // no interrupts
                .TXCIE0 = 0, // no interrupts
                .RXCIE0 = 0, // no interrupts
            });
            USART0.UCSR0C.write(.{
                .UCPOL0 = 0, // async mode
                .UCSZ0 = @as(u2, @truncate((ucsz & 0x03) >> 0)),
                .USBS0 = usbs,
                .UPM0 = upm,
                .UMSEL0 = umsel,
            });

            USART0.UBRR0.modify(ubrr_val);

            return Self{};
        }

        pub fn canWrite(self: Self) bool {
            _ = self;
            return (USART0.UCSR0A.read().UDRE0 == 1);
        }

        pub fn tx(self: Self, ch: u8) void {
            while (!self.canWrite()) {} // Wait for Previous transmission
            USART0.UDR0.* = ch; // Load the data to be transmitted
        }

        pub fn canRead(self: Self) bool {
            _ = self;
            return (USART0.UCSR0A.read().RXC0 == 1);
        }

        pub fn rx(self: Self) u8 {
            while (!self.canRead()) {} // Wait till the data is received
            return USART0.UDR0.*; // Read received data
        }
    };
}
