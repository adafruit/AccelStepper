# Makefile
#
# Makefile for the Arduino AccelStepper project
#
# Author: Mike McCauley (mikem@open.com.au)
# Copyright (C) 2010 Mike McCauley
# $Id: Makefile,v 1.1 2010/04/25 02:21:18 mikem Exp mikem $

PROJNAME = AccelStepper
# Dont forget to also change the version at the top of AccelStepper.h:
DISTFILE = $(PROJNAME)-1.3.zip

all:	doxygen dist upload

doxygen: 
	doxygen project.cfg


ci:
	ci -l `cat MANIFEST`

dist:	
	(cd ..; zip $(PROJNAME)/$(DISTFILE) `cat $(PROJNAME)/MANIFEST`)

upload:
	scp $(DISTFILE) doc/*.html doc/*.gif doc/*.png doc/*.css doc/*.pdf server2:/var/www/html/mikem/arduino/$(PROJNAME)
