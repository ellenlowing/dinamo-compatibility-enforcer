#MenuTitle: OpenCorner
# -*- coding: utf-8 -*-

import math

glyph = Font.selectedLayers[0].parent
layer = glyph.layers[0]
allsegments = layer.paths[0].segments

# for path in layer.paths:
# 	isPrevCornerOpened = False
# 	for node in path.nodes:
# 		if isCorner(node):
# 			if isPrevCornerOpened:
# 				print "this is the 2nd node in open corner, skipping"
# 				isPrevCornerOpened = False
# 				continue
#
# 			if isCornerOpened(node):
# 				isPrevCornerOpened = True
# 				print "open corner found!"
# 			else:
# 				openCorner(node)
# 				print "opening new corner"

def lerp(x, inMin, inMax, outMin, outMax):
	return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def isHorizontal(p0, p1):
	return p0.y-p1.y==0

def isVertical(p0, p1):
	return p0.x-p1.x==0

def intersectsBetweenHorizontalAndVertical(seg0, seg1):
	horizontal = seg0 if isHorizontal(seg0[0],seg0[1]) else seg1
	vertical = seg1 if horizontal == seg0 else seg0
	print horizontal[0], horizontal[1]
	xidx = 1 if horizontal[1].x > horizontal[0].x else 0
	yidx = 1 if vertical[1].y > vertical[0].y else 0
	return horizontal[1-xidx].x <= vertical[0].x <= horizontal[xidx].x and vertical[1-yidx].y <= horizontal[0].y <= vertical[yidx].y


# TODO: if open corner is found next node in paths should be skipped
def isCornerOpened(node):
	if isCorner(node) and isCorner(node.nextNode):
		seg0 = findSegmentsContainingNode(allsegments, node)[0]
		seg1 = findSegmentsContainingNode(allsegments, node.nextNode)[1]

		if len(seg0) is 2 and len(seg1) is 2:
			# 2 straight lines
			p0 = seg0[0]
			p1 = seg0[1]
			p2 = seg1[0]
			p3 = seg1[1]
			if (isHorizontal(p0,p1) and isHorizontal(p2,p3)) or (isVertical(p0,p1) and isVertical(p2,p3)):
				# no intersection for parallel segments
				return False

			elif (isHorizontal(p0,p1) and isVertical(p2,p3)) or (isVertical(p0,p1) and isHorizontal(p2,p3)):
				return intersectsBetweenHorizontalAndVertical(seg0, seg1)

			else:
				# TODO: slanted straight lines
				print "slanted lines"
# 				ix = ((p1.x*p0.y-p0.x*p1.y)*(p3.x-p2.x) - (p3.x*p2.y-p2.x*p3.y)*(p1.x-p0.x)) / ((p1.x-p0.x)*(p3.y-p2.y) - (p3.x-p2.x)*(p1.y-p0.y))
#  				iy = (p1.y-p0.y)*(ix-p0.x)/(p1.x-p0.x)+p0.y
# 				print ix,iy

		else:
			# TODO: find intersection between line and bezier curve
			curve = seg0 if len(seg0) == 4 else seg1
			line = seg0 if len(seg0) == 2 else seg1
			px = [curve[0].x, curve[1].x, curve[2].x, curve[3].x]
			py = [curve[0].y, curve[1].y, curve[2].y, curve[3].y]
			lx = [line[0].x, line[1].x]
			ly = [line[0].y, line[1].y]
			intersections = computeIntersections(px,py,lx,ly)
			print intersections
			return True


def openCorner(node):
	# TODO: need to check if node has already been opened,
	# or like if its corresponding master has opened its node

	# TODO: need to store all new nodes and old corner nodes in the process and
	# add and delete all at once in the end, because if consecutive nodes would affect
	# open corner nodes calculation

	index = node.index
	path = node.parent

	# get 2 segments: one that joins the prev node, and one that joins the next
	segments = findSegmentsContainingNode(allsegments, node)

	# remove selected node
	del path.nodes[node.index]

	# insert 2 corner nodes at the end of each extended segment
	for i,segment in enumerate(segments):
		newNode = GSNode()
		nx = node.x
		ny = node.y
		if len(segment) == 2:
			newNode.type = GSLINE
			prevNode = segment[0]
			nextNode = segment[1]
			direction = -1 if i == 0 else 1
			if prevNode.y - nextNode.y == 0:
				# perfectly horizontal
				if prevNode.x > nextNode.x:
					nx += direction * 43
				else:
					nx -= direction * 43
			elif prevNode.x - nextNode.x == 0:
				# perfectly vertical
				if prevNode.y > nextNode.y:
					ny += direction * 43
				else:
					ny -= direction * 43
			else:
				slope = (prevNode.y - nextNode.y) / (prevNode.x - nextNode.x)
				dist = 66
				nx = node.x + dist / math.sqrt(slope*slope+1) * direction
				ny = slope*(nx - node.x) + node.y

		else:
			newNode.type = GSCURVE if i == 0 else GSLINE
			t = 1.2 if i == 0 else -0.2
			p0 = segment[0]
			p1 = segment[1]
			p2 = segment[2]
			p3 = segment[3]
			p1idx = index-2 if i == 0 else index+1
			p2idx = index-1 if i == 0 else index+2

			# update p3 position if t = 1.2, else update p0
			nx = math.pow(1-t, 3)*p0.x + 3*math.pow(1-t,2)*t*p1.x + 3*(1-t)*math.pow(t,2)*p2.x + math.pow(t,3)*p3.x
			ny = math.pow(1-t, 3)*p0.y + 3*math.pow(1-t,2)*t*p1.y + 3*(1-t)*math.pow(t,2)*p2.y + math.pow(t,3)*p3.y
			if i == 0:
				p3 = NSMakePoint(nx,ny)
			else:
				p0 = NSMakePoint(nx,ny)

			# update p1, p2 position with new p3,p0 through interpolation
			b25 = getBezierCoord(0.25, segment)
			b75 = getBezierCoord(0.75, segment)
			t25 = lerp(0.25, 0, t, 0, 1) if i == 0 else lerp(0.25, t, 1, 0, 1)
			t75 = lerp(0.75, 0, t, 0, 1) if i == 0 else lerp(0.75, t, 1, 0, 1)

			# components used to solve cubic bezier equations
			p0x_25 = math.pow(1-t25,3)*p0.x
			p0y_25 = math.pow(1-t25,3)*p0.y
			p0x_75 = math.pow(1-t75,3)*p0.x
			p0y_75 = math.pow(1-t75,3)*p0.y
			p3x_25 = math.pow(t25,3)*p3.x
			p3y_25 = math.pow(t25,3)*p3.y
			p3x_75 = math.pow(t75,3)*p3.x
			p3y_75 = math.pow(t75,3)*p3.y
			p2x = ((b75.x - p0x_75 - p3x_75) / (3 * (1-t75) * math.pow(t75,2)) - ( (1-t75)*(b25.x - p0x_25 - p3x_25) / (3 * t25 * math.pow(1-t25,2) * t75) )) / (1 - t25*(1-t75)/(t75*(1-t25)))
			p2y = ((b75.y - p0y_75 - p3y_75) / (3 * (1-t75) * math.pow(t75,2)) - ( (1-t75)*(b25.y - p0y_25 - p3y_25) / (3 * t25 * math.pow(1-t25,2) * t75) )) / (1 - t25*(1-t75)/(t75*(1-t25)))
			p1x = (b25.x - p0x_25 - p3x_25 - 3*(1-t25)*math.pow(t25,2)*p2x) / (3 * math.pow(1-t25,2) * t25)
			p1y = (b25.y - p0y_25 - p3y_25 - 3*(1-t25)*math.pow(t25,2)*p2y) / (3 * math.pow(1-t25,2) * t25)

			path.nodes[p1idx].position = NSMakePoint(p1x,p1y)
			path.nodes[p2idx].position = NSMakePoint(p2x,p2y)

		newNode.position = NSMakePoint(nx, ny)
		path.insertNode_atIndex_(newNode, index+i)

def computeIntersections(px, py, lx, ly):
	X = [-1,-1]
	A = ly[1]-ly[0]
	B = lx[0]-lx[1]
	C = lx[0]*(ly[0]-ly[1])+ly[0]*(lx[1]-lx[0])

	bx = bezierCoeffs(px[0], px[1], px[2], px[3])
	by = bezierCoeffs(py[0], py[1], py[2], py[3])

	P = []
	P.append(A*bx[0]+B*by[0])
	P.append(A*bx[1]+B*by[1])
	P.append(A*bx[2]+B*by[2])
	P.append(A*bx[3]+B*by[3] + C)

	r = cubicRoots(P)
	intersections = []

	for i in range(0,3):
		t = r[i]
		X[0] = bx[0]*t*t*t + bx[1]*t*t + bx[2]*t + bx[3]
		X[1] = by[0]*t*t*t + by[1]*t*t + by[2]*t + by[3]

		s = 0
		if lx[1]-lx[0] != 0:
			s = (X[0]-lx[0])/(lx[1]-lx[0])
		else:
			s = (X[1]-ly[0])/(ly[1]-ly[0])

		if t<0 or t>1.0 or s<0 or s>1.0:
			X[0] = -100
			X[1] = -100

		intersection = NSMakePoint(X[0],X[1])
		intersections.append(intersection)

	return intersections


def bezierCoeffs(p0, p1, p2, p3):
	Z = []
	Z.append(-p0 + 3*p1 - 3*p2 + p3)
	Z.append(3*p0 - 6*p1 + 3*p2)
	Z.append(-3*p0 + 3*p1)
	Z.append(p0)
	return Z

def sgn(x):
	return -1 if x < 0 else 1

def cubicRoots(P):
	a = P[0]
	b = P[1]
	c = P[2]
	d = P[3]
	A = b/a
	B = c/a
	C = d/a
	Q = (3*B - math.pow(A,2))/9
	R = (9*A*B - 27*C - 2*math.pow(A,3))/54
	D = math.pow(Q,3) + math.pow(R,2)
	t=[-1,-1,-1]

	if D >= 0:
		S = sgn(R + math.pow(D,0.5))*math.pow(math.abs(R + math.pow(D,0.5)), 1/3)
		T = sgn(R - math.pow(D,0.5))*math.pow(math.abs(R - math.pow(D,0.5)), 1/3)

		t[0]=(-A/3 + (S+T))
		t[1]=(-A/3 - (S+T)/2)
		t[2]=(-A/3 - (S+T)/2)
		im = math.abs(math.pow(3*(S-T)/2,0.5))

		if im != 0:
			t[1]=-1
			t[2]=-1
	else:
		th = math.acos(R/math.pow(-math.pow(Q,3),0.5))
		t[0] = 2*math.pow(-Q,0.5)*math.cos(th/3)-A/3
		t[1] = 2*math.pow(-Q,0.5)*math.cos((th+2*math.pi)/3)-A/3
		t[2] = 2*math.pow(-Q,0.5)*math.cos((th+4*math.pi)/3)-A/3
		im = 0.0

	for i in range(0,3):
		if t[i] < 0 or t[i] > 1.0:
			t[i] = -1

	t = sortSpecial(t)
	return t

def sortSpecial(a):
	flip=None
	temp=None

	while True:
		flip=False
		for i in range(0,len(a)-1):
			if (a[i+1]>=0 and a[i]>a[i+1]) or (a[i]<0 and a[i+1]>=0):
				flip = True
				temp = a[i]
				a[i] = a[i+1]
				a[i+1] = temp

		if not flip:
			break

	return a

def getBezierCoord(t, segment):
	p0 = segment[0]
	p1 = segment[1]
	p2 = segment[2]
	p3 = segment[3]
	x = math.pow(1-t, 3)*p0.x + 3*math.pow(1-t,2)*t*p1.x + 3*(1-t)*math.pow(t,2)*p2.x + math.pow(t,3)*p3.x
	y = math.pow(1-t, 3)*p0.y + 3*math.pow(1-t,2)*t*p1.y + 3*(1-t)*math.pow(t,2)*p2.y + math.pow(t,3)*p3.y
	return NSMakePoint(x,y)

# Detect if node is a corner
def isCorner(node):
	return not node.smooth and node.type is not "offcurve"

# Check if node is last in path
def isLastNode(node):
	path = node.parent
	return node.index == len(path.nodes)-1

# Find segments containing given node
def findSegmentsContainingNode(pathsegments, node):
	segments = []
	for segment in pathsegments:
		for point in segment:
			if point.x == node.x and point.y == node.y:
				segments.append(segment)
	if isLastNode(node):
		segments.reverse()
	return segments

isPrevCornerOpened = False
for selectedNode in Layer.selection:
	if isCorner(selectedNode):
		if isPrevCornerOpened:
			print "this is the 2nd node in open corner, skipping"
			isPrevCornerOpened = False
			continue

		if isCornerOpened(selectedNode):
			isPrevCornerOpened = True
			print "open corner found!"
		else:
			openCorner(selectedNode)
			print "opening new corner..."
