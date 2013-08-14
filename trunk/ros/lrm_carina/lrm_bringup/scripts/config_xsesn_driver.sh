#-m, --output-mode=MODE
#        Mode of the device selecting the information to output.
#        This is required for 'configure' command. If it is not present
#		in 'echo' command, the configuration will be read from the
#        device.
#        MODE can be either the mode value in hexadecimal, decimal or
#        binary form, or a string composed of the following characters
#        (in any order):
#                t       temperature, [0x0001]
#                c       calibrated data, [0x0002]
#                o       orientation data, [0x0004]
#                a       auxiliary data, [0x0008]
#                p       position data (requires MTi-G), [0x0010]
#                v       velocity data (requires MTi-G), [0x0020]
#                s       status data, [0x0800]
#                g       raw GPS mode (requires MTi-G), [0x1000]
#                r       raw (incompatible with others except raw GPS),
#                        [0x4000]
#        For example, use "--output-mode=so" to have status and
#        orientation data.
#-s, --output-settings=SETTINGS
#        Settings of the device.
#        This is required for 'configure' command. If it is not present
#        in 'echo' command, the configuration will be read from the
#        device.
#        SETTINGS can be either the settings value in hexadecimal,
#        decimal or binary form, or a string composed of the following
#        characters (in any order):
#                t       sample count (excludes 'n')
#                n       no sample count (excludes 't')
#                q       orientation in quaternion (excludes 'e' and 'm')
#                e       orientation in Euler angles (excludes 'm' and
#                        'q')
#                m       orientation in matrix (excludes 'q' and 'e')
#                A       acceleration in calibrated data
#                G       rate of turn in calibrated data
#                M       magnetic field in calibrated data
#                i       only analog input 1 (excludes 'j')
#                j       only analog input 2 (excludes 'i')
#                N       North-East-Down instead of default: X North Z up
#        For example, use "--output-settings=tqMAG" for all calibrated
#        data, sample counter and orientation in quaternion.


rosrun xsens_driver mtdevice.py --device=auto --inspect
rosrun xsens_driver mtdevice.py --device=auto --reset
rosrun xsens_driver mtdevice.py --device=auto --inspect
rosrun xsens_driver mtdevice.py --device=auto --configure --output-mode=tcoapvs --output-setting=tqAGMN --xkf-scenario=2
rosrun xsens_driver mtdevice.py --device=auto --inspect

