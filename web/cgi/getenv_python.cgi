#! /usr/bin/python3.7
# -*- coding: utf-8 -*-

# Import modules for CGI handling 
import cgi, cgitb 
import serial


print ("Content-Type: text/html")
print ("")

# Create instance of FieldStorage 
form = cgi.FieldStorage() 

# Get data from fields
valeur = form.getvalue('note')
# time = form.getvalue('time')

if (valeur!=None):
	ser=serial.Serial('/dev/ttyAMA0',baudrate=9600,timeout=10)
	# ser.write(114514)
	ser.write([65+int(float(valeur))])
	# ser.write(int(1000*float(time)))
	

