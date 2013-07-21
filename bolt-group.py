# This is a python program that takes user input of forces, moments, and 
# bolt position, and generates bolt loads on a bolt group
# All values, fx, fy, mz, xf, yf are floats, except for boltx and bolt
# which are arrays
#
# This function returns
# boltx, bolty,px,py,pxy,ex,ey,em
# Where boltx and bolty were the given bolt positions
# px and py are arrays of bolt loads in the x and y axes
# pxy is an array of bolt shear loads
# ex,ey, and em are errors in the input loads vs. output loads
#

import math
def boltgrp(fx,fy,mz,xf,yf,boltx,bolty):

	if (len(boltx)!=len(bolty)):
		return -1

	number = len(boltx)
	#bolt CGs
	cgx=math.fsum(boltx)/number	
	cgy=math.fsum(bolty)/number

	#Moment due to forces and total moment
	mzfx=fx*(cgy-yf)
	mzfy=-fy*(cgx-xf)
	mztot=mz+mzfx+mzfy

	#direct shear loads
	pfx=fx/number
	pfy=fy/number

	#radii to compute direction cosines
	#radii squared, and radii
	rx = []
	ry = []
	rsquared = []
	r = []

	for i in range(0,number):
		rx.append(boltx[i]-cgx)
		ry.append(bolty[i]-cgy)
		rsquared.append(rx[i]**2 + ry[i]**2)
		r.append(math.sqrt(rsquared[i]))

	sumr2 = math.fsum(rsquared)
	#radius/sum of radii squared to determine tangential force
	#tangential force calculation
	rdsumr2 = []
	ft = []

	for i in range(0,number):
		rdsumr2.append(r[i]/sumr2)
		ft.append(rdsumr2[i]*mztot)
	
	#determine components of tangential forces in x,y coordinates
	rxdr=[]
	rydr=[]
	pmx=[]
	pmy=[]
	#determine total x, y forces and shear on bolts
	px=[]
	py=[]
	pxy=[]

	for i in range(0,number):
		rxdr.append(rx[i]/r[i])
		rydr.append(ry[i]/r[i])
		pmx.append(-ft[i]*rydr[i])
		pmy.append(ft[i]*rxdr[i])
		px.append(pmx[i]+pfx)
		py.append(pmy[i]+pfy)
		pxy.append(math.sqrt(py[i]**2 + px[i]**2))

	#find resultant total loads and moments to check vs. input
	sx=math.fsum(px)
	sy=math.fsum(py)
	mpx=[]
	mpy=[]

	for i in range(0,number):
		mpx.append(-px[i]*(bolty[i]-yf))
		mpy.append(py[i]*(boltx[i]-xf))

	#check resulting moments calculated at input loads vs input
	rmx=math.fsum(mpx)
	rmy=math.fsum(mpy)
	rmtot=rmx+rmy
	ex=math.fabs(sx-fx)/math.fabs(fx)
	ey=math.fabs(sy-fy)/math.fabs(fy)
	em=math.fabs(rmtot-mz)/math.fabs(mz)
	return boltx, bolty,px,py,pxy,ex,ey,em
