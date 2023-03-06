from caproto.server import PVGroup, ioc_arg_parser, pvproperty, run
from caproto import ChannelType
import asyncio
from textwrap import dedent

RANGE_OPTIONS=['2E-2', '2E-3', '2E-4', '2E-5', '2E-6', '2E-7', '2E-8', '2E-9']

class Keithley6487(PVGroup):
    """
    A class to control Omega CN8PT temperature controllers
    """
    
    CURRENT = pvproperty(value=0.0, record='ai', doc="Current")
    COUNT_TIME = pvproperty(value=1.0, record='ai')
    RANGE = pvproperty(value="", dtype=str, report_as_string=True)
    sensor = pvproperty(value="",
                        enum_strings=("thermocouple",
                                      "rtd",
                                      "process input",
                                      "thermistor"),
                        record="mbbi",
                        dtype=ChannelType.ENUM,
                        doc="The sensor type of the contoller")
    si1 = pvproperty(value="", dtype=str, report_as_string=True, doc="Sensor info 1")
    si2 = pvproperty(value="", dtype=str, report_as_string=True, doc="Sensor info 2")
    setpoint = pvproperty(value=0.0, record='ai', doc="control setpoint")
    output_chan = pvproperty(value=3, doc="output channel")
    output_mode = pvproperty(value="off", enum_strings=("off",
                                                        "pid",
                                                        "on-off",
                                                        "retransmit",
                                                        "alarm1",
                                                        "alarm2",
                                                        "ramp soak RE",
                                                        "ramp soak SE"),
                             record="mbbi",
                             dtype=ChannelType.ENUM,
                             doc="output mode")
    output_range = pvproperty(value=0, enum_strings=("0-10V", "0-5V", "0-20V", "4-20V", "0-24V"),
                              record="mbbi",
                              dtype=ChannelType.ENUM,
                              doc="Output range")
    output_type = pvproperty(value="No Output", report_as_string=True, dtype=str)
    pid_hi_lim = pvproperty(value=100, doc="maximum output percentage")

    def __init__(self, *args, address="10.66.50.95", port=2000, **kwargs):
        self.address = address
        self.port = port
        self.ioLock = asyncio.Lock()
        super().__init__(*args, **kwargs)

    async def __ainit__(self, async_lib):
        # Turn on echo so that we know if device is communicating
        await self.write_and_read("W330", "00011")

    @ACQUIRE.putter
    async def ACQUIRE(self, instance, value):
        if value != 0:
            reading = await self.query("READ")
            await self.CURRENT.write(reading)
        return value

    @COUNT_TIME.putter
    async def COUNT_TIME(self, instance, value):
        low=.01
        high=1
        PLC = 60.0*value
        if PLC < low:
            PLC = low
        if PLC > high:
            PLC = high
        
    @RANGE.putter
    async def RANGE(self, instance, value):
        if value in RANGE_OPTIONS:
            await self.write("RANGE", value)

    async def query(self, command, value=None):
        async with self.ioLock:
            reader, writer = await asyncio.open_connection(self.address, self.port)

            termination = "\r"
            if value is None:
                msg = f"{command}?{termination}"
            else:
                msg = f"{command} {value}?{termination}"
            print(msg)
            writer.write(msg.encode())
            await writer.drain()
            data = await reader.read(100)
            response = data.decode().rstrip()[len(command):]
            writer.close()
            await writer.wait_closed()
        return response
    
    async def write(self, command, value=None):
        async with self.ioLock:
            reader, writer = await asyncio.open_connection(self.address, self.port)

            termination = "\r"
            if value is None:
                msg = f"{command}{termination}"
            else:
                msg = f"{command} {value}{termination}"
            print(msg)
            writer.write(msg.encode())
            await writer.drain()
            writer.close()
            await writer.wait_closed()
        return

    @sensor.startup
    async def sensor(self, instance, async_lib):
        print("running sensor startup hook")
        sensor_config = await self.write_and_read("R100")
        print(sensor_config)
        stype, si1, si2 = self.parse_sensor_config(sensor_config)
        await instance.write(stype)
        await self.si1.write(si1)
        await self.si2.write(si2)

    def parse_sensor_config(self, sensor_config):
        stype = int(sensor_config[0])
        si1i = int(sensor_config[1])
        si2i = int(sensor_config[2])
        si1 = si1_lookup.get(stype, {}).get(si1i, "")
        si2 = si2_lookup.get(stype, {}).get(si2i, "")
        return stype, si1, si2
      
    @output_chan.startup
    async def output_chan(self, instance, async_lib):
        await self.update_output_config()

    async def update_output_config(self, chan=None):
        if chan is None:
            chan = self.output_chan.value
        mode = int(await self.write_and_read("R600", chan))
        await self.output_mode.write(mode)
        outtype = await self.write_and_read("G601", chan)
        await self.output_type.write(output_type_lookup.get(outtype, "No Output"))
        outrange = int(await self.write_and_read("R660", chan))
        await self.output_range.write(outrange)

    @output_chan.putter
    async def output_chan(self, instance, value):
        await self.update_output_config(value)

    @output_mode.putter
    async def output_mode(self, instance, value):
        chan = self.output_chan.value
        rawval = instance.get_raw_value(value)
        cmd = f"{chan}{rawval}"
        await self.write_and_read("W600", cmd)

    @output_range.putter
    async def output_range(self, instance, value):
        chan = self.output_chan.value
        rawval = instance.get_raw_value(value)
        cmd = f"{chan}{rawval}"
        await self.write_and_read("W660", cmd)
        
    @setpoint.startup
    async def setpoint(self, instance, async_lib):
        sp = await self.write_and_read("R400")
        sp = float(sp.lstrip("+"))
        await instance.write(sp)

    @setpoint.putter
    async def setpoint(self, instance, value):
        await self.write_and_read("W400", value)

    @pid_hi_lim.putter
    async def pid_hi_lim(self, instance, value):
        hex_val = f"{value:02X}"
        await self.write_and_read("W502", hex_val)
    
    @temperature.scan(period=2, use_scan_field=True)
    async def temperature(self, instance, async_lib):
        r = await self.write_and_read("G110")
        t = float(r.lstrip("+"))
        await instance.write(t)

    
    
if __name__ == "__main__":
    ioc_options, run_options = ioc_arg_parser(default_prefix="cn8pt:",
                                              desc = dedent(CN8PT.__doc__),
                                              supported_async_libs=('asyncio',))
    ioc = CN8PT(**ioc_options)
    run(ioc.pvdb, startup_hook=ioc.__ainit__, **run_options)
