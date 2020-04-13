#MenuTitle: OpenCorner
# -*- coding: utf-8 -*-

for selectedNode in Layer.selection:
	if isCorner(selectedNode):
		print selectedNode
		openCorner(selectedNode)

glyph = Font.selectedLayers[0].parent
layer = glyph.layers[0]
allsegments = layer.paths[0].segments

def openCorner(node):
	# TODO: need to check if node has already been opened,
	# or like if its corresponding master has opened its node

	# TODO: need to store all new nodes and old corner nodes in the process and
	# add and delete all at once in the end, because if consecutive nodes would affect
	# open corner nodes calculation

	index = node.index
	path = node.parent
	nodeA = GSNode()
	nodeB = GSNode()

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
			t = 1.1 if i == 0 else -0.1
			p0 = segment[0]
			p1 = segment[1]
			p2 = segment[2]
			p3 = segment[3]

			# update p3 position
			nx = math.pow(1-t, 3)*p0.x + 3*math.pow(1-t,2)*t*p1.x + 3*(1-t)*math.pow(t,2)*p2.x + math.pow(t,3)*p3.x
			ny = math.pow(1-t, 3)*p0.y + 3*math.pow(1-t,2)*t*p1.y + 3*(1-t)*math.pow(t,2)*p2.y + math.pow(t,3)*p3.y

			# TODO: update p1, p2 position with new p3



		newNode.position = NSMakePoint(nx, ny)
		if i == 0:
			nodeA = newNode
		else:
			nodeB = newNode


	path.insertNode_atIndex_(nodeA, index)
	path.insertNode_atIndex_(nodeB, index)



# Detect if node is a corner
def isCorner(node):
	return not node.smooth and node.type is not "offcurve"


# Find segments containing given node
def findSegmentsContainingNode(pathsegments, node):
	segments = []
	for segment in pathsegments:
		for point in segment:
			if point.x == node.x and point.y == node.y:
				segments.append(segment)
	return segments
