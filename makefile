.PHONY: PI_install PI_uninstall all BBB_install BBB_uninstall

all:
	$(MAKE) -C sdr_record

PI_install: sdr_record/sdr_record autostart/rctstart gps_logger/gps_logger.py autostart/parser.sh getRunNum.py
	cp autostart/rctstart /etc/init.d/
	update-rc.d rctstart defaults

PI_uninstall:
	update-rc.d rctstart remove
	rm /etc/init.d/rctstart

BBB_install: sdr_record/sdr_record autostart/rctstart gps_logger/gps_logger.py autostart/parser.sh getRunNum.py
	cp autostart/rctstart /etc/init.d/
	update-rc.d rctstart defaults
	mkdir /media/RAW_DATA/
	cp autostart/mount_RAW_DATA /etc/init.d/
	update-rc.d mount_RAW_DATA defaults
