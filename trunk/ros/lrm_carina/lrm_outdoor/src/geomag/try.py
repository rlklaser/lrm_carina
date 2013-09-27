#!/usr/bin/python

import geomag


gm = geomag.GeoMag("WMM.COF")
#mag = gm.calc(43.411454, -80.472708)
mag = gm.calc(-22.411454, -44.472708)
print mag.dec
