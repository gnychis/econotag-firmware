#!/usr/bin/ruby

path=nil
path="/dev/ttyUSB1" if(File.exist?("/dev/ttyUSB1"))
path=Dir.glob("/dev/tty.usb*B") if(path.nil?)

puts "Device path: #{path}"

system("./libmc1322x/tools/ftditools/bbmc -l redbee-econotag erase")
system("./libmc1322x/tools/mc1322x-load.pl -b 200 -f libmc1322x/tests/flasher_redbee-econotag.bin -s #{ARGV[0]} -t #{path}")
