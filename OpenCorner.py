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
		print seg0, seg1

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
				# slanted straight lines
				print "slanted lines"

		else:
			# find intersection between line and bezier curve
			print "find intersection between line and bezier curve"
			
# 			# calculate intersection between Bezier and straight
# 		ix = ((p1.x*p0.y-p0.x*p1.y)*(p3.x-p2.x) - (p3.x*p2.y-p2.x*p3.y)*(p1.x-p0.x)) / ((p1.x-p0.x)*(p3.y-p2.y) - (p3.x-p2.x)*(p1.y-p0.y))
# 		iy = (p1.y-p0.y)*(ix-p0.x)/(p1.x-p0.x)+p0.y
# 		print ix,iy

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
