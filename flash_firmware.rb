#!/usr/bin/ruby

path=nil
path="/dev/ttyUSB1" if(File.exist?("/dev/ttyUSB1"))
path=Dir.glob("/dev/tty.usb*B") if(path.nil?)

`sudo ./libmc1322x/tools/ftditools/bbmc -l redbee-econotag erase`
`sudo ./libmc1322x/tools/mc1322x-load.pl -f libmc1322x/tests/flasher_redbee-econotag.bin -s $1 -t #{path}`
