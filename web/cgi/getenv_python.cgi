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
interval = form.getvalue('interval')

if (valeur!=None):
	ser=serial.Serial('/dev/ttyAMA0',baudrate=9600,timeout=10)
	# ser.write([255])
	# ser.write([255,int(float(valeur)),int(1000*float(interval))])
	ser.write([int(float(valeur))])
	# ser.write([int(float(interval))])

