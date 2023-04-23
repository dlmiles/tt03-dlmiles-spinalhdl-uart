#!/usr/bin/python3
#
#
import os
import sys
import re
import inspect

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ClockCycles
from cocotb.wavedrom import trace
from cocotb.binary import BinaryValue


def try_integer(v, default_value=None):
    if type(v) is int:
        return v
    if v.is_resolvable:
        return v.integer
    if default_value is not None:
        return default_value
    return v

def try_binary(v, width=None):
    if type(v) is BinaryValue:
        return v
    if type(v) is str:
        return v
    if width is None:
        return BinaryValue(v)
    else:
        return BinaryValue(v, n_bits=width)

# Useful when you want a particular format, but only if it is a number
#  try_decimal_format(valye, '3d')
def try_decimal_format(v, fmt=None):
    #print("try_decimal_format(v={} {}, fmt={} {})".format(v, type(v), fmt, type(fmt)))
    if fmt is not None and type(v) is int:
        fmtstr = "{{0:{}}}".format(fmt)
        #print("try_decimal_format(v={} {}, fmt={} {})  fmtstr=\"{}\" => \"{}\"".format(v, type(v), fmt, type(fmt), fmtstr, fmtstr.format(v)))
        return fmtstr.format(v)
    return "{}".format(v)

def try_compare_equal(a, b):
    a_s = str(try_binary(a))	# string
    b_s = str(try_binary(b))
    rv = a_s == b_s
    #print("{} {} == {} {} => {}".format(a, a_s, b, b_s, rv))
    return rv

def try_name(v):
    if v is None:
        return None
    if hasattr(v, '_name'):
        return v._name
    return str(v)

def try_path(v):
    if v is None:
        return None
    if hasattr(v, '_path'):
        return v._path
    return str(v)

def try_value(v):
    if v is None:
        return None
    if hasattr(v, 'value'):
        return v.value
    return str(v)

def report_resolvable(dut, pfx = None, node = None, depth = None):
    if depth is None:
        depth = 3
    if depth < 0:
        return
    if node is None:
        node = dut
        if pfx is None:
            pfx = "DUT."
    if pfx is None:
        pfx = ""
    for design_element in node:
        if isinstance(design_element, cocotb.handle.ModifiableObject):
            dut._log.info("{}{} = {}".format(pfx, design_element._name, design_element.value))
        elif isinstance(design_element, cocotb.handle.HierarchyObject) and depth > 0:
            report_resolvable(dut, pfx + try_name(design_element) + '.', design_element, depth - 1)	# recurse 
        else:
            dut._log.info("{}{} = {}".format(pfx, try_name(design_element), type(design_element)))
    pass


# Does not nest
def design_element_internal(dut_or_node, name):
    #print("design_element_internal(dut_or_node={}, name={})".format(dut_or_node, name))
    for design_element in dut_or_node:
        #print("design_element_internal(dut_or_node={}, name={}) {} {}".format(dut_or_node, name, try_name(design_element), design_element))
        if design_element._name == name:
            return design_element
    return None

# design_element(dut, 'module1.module2.signal')
def design_element(dut, name):
    names = name.split('.')	# will return itself if no dot
    #print("design_element(name={}) {} len={}".format(name, names, len(names)))
    node = dut
    for name in names:
        node = design_element_internal(node, name)
        if node is None:
            return None
    return node

def design_element_exists(dut, name):
    return design_element(dut, name) is not None


###
###
##################################################################################################################################
###
###

async def send_in7_oneedge(dut, in7):
    in7_before = dut.in7.value
    out8_before = dut.out8.value
    clk_before, = dut.clk.value
    in8 = try_integer(dut.in7.value, 0) << 1 | try_integer(dut.clk.value, 0)	# rebuild for log output
    dut.in7.value = in7
    if dut.clk.value:
        await FallingEdge(dut.clk)
    else:
        await RisingEdge(dut.clk)
    # Try to report non-clock state changes
    out8_equal = try_compare_equal(out8_before, dut.out8.value)
    if True or in7_before != in7 or not out8_equal:
        out8_desc = "SAME" if(out8_equal) else "CHANGED"
        dut._log.info("dut clk={} in7={} {} in8={} {}  =>  out8={} {} => {} {}  [{}]".format(
            clk_before,
            try_binary(dut.in7.value, width=7),  try_decimal_format(try_integer(dut.in7.value), '3d'),
            try_binary(in8, width=8),            try_decimal_format(try_integer(in8), '3d'),
            try_binary(out8_before, width=8),    try_decimal_format(try_integer(out8_before), '3d'),
            try_binary(dut.out8.value, width=8), try_decimal_format(try_integer(dut.out8.value), '3d'),
            out8_desc))

async def send_in7(dut, in7):
    in8 = try_integer(dut.in7.value, 0) << 1 | try_integer(dut.clk.value, 0)	# rebuild for log output
    dut._log.info("dut out8={} in7={} in8={}".format(dut.out8.value, dut.in7.value, in8))
    await FallingEdge(dut.clk)
    dut.in7.value = in7
    await RisingEdge(dut.clk)
    dut._log.info("dut out8={} in7={} in8={}".format(dut.out8.value, dut.in7.value, in8))

async def send_in8(dut, in8):
    in7 = (in8 >> 1) & 0x7f
    await send_in7(dut, in7)

async def send_in8_oneedge(dut, in8):
    want_clk = in8 & 0x01
    # The rom.txt scripts expect to drive CLK as well, so we need to align edge so current
    #  state is mismatched
    if dut.clk.value and want_clk != 0:
        if dut.clk.value:
            dut._log.warning("dut ALIGN INSERT EDGE: Falling (clk={}, want_clk={})".format(dut.clk.value, want_clk))
            await FallingEdge(dut.clk)
        else:
            dut._log.warning("dut ALIGN INSERT EDGE: Rising (clk={}, want_clk={})".format(dut.clk.value, want_clk))
            await RisingEdge(dut.clk)
    in7 = (in8 >> 1) & 0x7f
    await send_in7_oneedge(dut, in7)

async def send_sequence_in8(dut, seq):
    for in8 in seq:
        await send_in8(dut, in8)

###
###
##################################################################################################################################
###
###

@cocotb.test()
async def test_uart(dut):
    dut._log.info("start")
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    report_resolvable(dut, 'initial ')

    await ClockCycles(dut.clk, 1)
    dut.in7.value = 0

    # This is a simulator async kludge, I need to break the X state (I don't care if with 0 or 1)
    # If it started up as a 1, we just clock a few more times (like 3 edges) to clear it.
    ele = design_element(dut, 'dut')
    print("A ele={} {}".format(try_path(ele), try_value(ele)))
    if ele:
        ele1 = design_element(ele, 'async_reset_ctrl')
        print("A ele1={} {}".format(try_path(ele1), try_value(ele1)))
        if ele1:
            ele2 = design_element(ele1, 'sim_reset')
            print("A ele2={} {}".format(try_path(ele2), try_value(ele2)))

    ele = design_element(dut, 'dut')
    print("B ele={} {}".format(try_path(ele), try_value(ele)))
    if ele:
        ele1 = design_element(ele, 'sim_reset')
        print("B ele1={} {}".format(try_path(ele1), try_value(el1)))

    ele = design_element(dut, 'dut.sim_reset')
    print("C ele={} {}".format(try_path(ele), try_value(ele)))
    ele = design_element(dut, 'dut.async_reset_ctrl')
    print("D ele={} {}".format(try_path(ele), try_value(ele)))
    ele = design_element(dut, 'dut.async_reset_ctrl.sim_reset')
    print("E ele={} {}".format(try_path(ele), try_value(ele)))

    await ClockCycles(dut.clk, 6)

    # signal not present during GL_TEST
    if design_element_exists(dut, 'dut.sim_reset'):
        dut.sim_reset.value = 0
        await ClockCycles(dut.clk, 1)
        dut.sim_reset.value = 1
        await ClockCycles(dut.clk, 1)
        dut.sim_reset.value = 0
        await ClockCycles(dut.clk, 1)
    elif design_element_exists(dut, 'dut.sync_reset'):
        # GL_TEST forcing a sync reset
        # We would not need to do this if we can instruct icarus to initialize the
        #   sr_latch to any state on powerup (not X state).
        dut.sync_reset.value = 0
        await ClockCycles(dut.clk, 1)
        dut.sync_reset.value = 1
        await ClockCycles(dut.clk, 1)
        dut.sync_reset.value = 0
        await ClockCycles(dut.clk, 1)

    await ClockCycles(dut.clk, 6)

    ele = design_element(dut, 'dut.sim_reset')
    print("F ele={} {}".format(try_path(ele), try_value(ele)))

    depth = None
    if 'GL_TEST' in os.environ:
        depth = 1
    report_resolvable(dut, depth=depth)

    # Perform reset sequence
    reset_seq = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    await send_sequence_in8(dut, reset_seq)

    # open rom.txt and run it
    filename = "../rom.txt"
    limit = 1024	# sys.maxsize
    if 'CI' in os.environ:
        limit = sys.maxsize	# CI no limit
    reader = RomReader(filename)
    count = 0
    while count < limit and reader.has_more():
        await send_in8_oneedge(dut, reader.next())
        count += 1
    dut._log.info("{} count={}".format(filename, count))

    report_resolvable(dut)


class RomReader():
    index = -1
    max_index = -1
    data = None

    def __init__(self, filename):
        fh = open(filename, "r")
        if fh is None:
            error()
        data = self.load_all(filename, fh)
        fh.close()
        
        if data is None:
            self.index = -1
            self.max_index = -1
            self.data = None
        else:
            self.max_index = len(data)
            self.index = 0
            self.data = data
        
        return None
        
    def max_index(self):
        return self.max_index
    
    def has_more(self):
        return self.index < self.max_index

    def next(self):
        if self.index < self.max_index:
            next_value = self.data[self.index]
            self.index += 1
            return next_value
        return None

    def rewind(self):
        self.index = 0
        return 0

    def parse_hex(self, s):
        if type(s) is int:	## FIXME maybe only if in range ?
            return s
        if type(s) is not str:
            return None
        if len(s) == 0:
            return None
        if not s.startswith("0x") and not s.startswith("0X"):
            s = "0x" + s
        #print("PHEX:{}:t={}:l={}".format(s, type(s), len(s)))
        try:
            v = int(s, 16)
            if type(v) is int and v >= 0x00 and v <= 0xff:
                #print("HEX:s={}:v={}:t={}".format(s, v, type(v)))
                return v
        except Exception as err:
            #print("EHEX:{}: Unexpected {}, {}".format(s, err, type(err)))
            pass
        return None

    def load_all(self, filename, fh):
        data = []
        line_number = 0
        pattern = re.compile(r"\s+")
        while True:
            line = fh.readline()
            if line is None or len(line) == 0:
                break
            line = line.rstrip()
            line_number += 1
            # Skip comment lines and blank lines
            if re.match(r'^\s*#', line) or re.fullmatch(r'^\s*$', line):
                #print("SKIP:{}:{}:{}".format(filename, line_number, line))
                continue
            words = pattern.split(line)
            #print("WORDS:{}:{}:l={}:t={}".format(filename, line_number, len(words), type(words)))
            for word in words:
                in8 = self.parse_hex(word)
                if in8 is None:
                    print("WARN:{}:{} invalid data: \"{}\"".format(filename, line_number, word))
                    continue
                data.append(in8)
    
        print("LOADED:{}:{} with {} bytes".format(filename, line_number, len(data)))
        return data
