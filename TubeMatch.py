#filename = [this file]
#exec(compile(open(filename).read(), filename, 'exec'))

import bpy
from mathutils import Vector
import mathutils
from math import pi, cos, sin, atan2
import numpy
import statistics
import bpy
from mathutils.geometry import intersect_point_line


def toVector(ar):
	return Vector((ar[0],ar[1],ar[2]));
def toArray(vec):
	return [vec.x,vec.y,vec.z];

def rotate(matrix, vector):
	vector = matrix @ Vector((vector.x,vector.y,vector.z,0));
	vector.resize_3d();
	return vector;

def toVector4(vec3, w=0):
	return Vector((vec3.x,vec3.y,vec3.z,w));

def toVector3(vec4):
	v = vec4.copy();
	v.resize_3d();
	return v;



class AABB:
	"""Axis-aligned bounding box"""
	vmin = Vector((float('inf'), float('inf'), float('inf')))
	vmax = Vector((float('-inf'), float('-inf'), float('-inf')))
	
	
	def includeOne(self, p):
		"""Grows the local instance so that it includes the given point
		
		Parameters
		----------
		p : Vector
			A point to include in the local bounding box
		"""
		self.vmin = Vector((
				min(self.vmin[0], p[0]),
				min(self.vmin[1], p[1]),
				min(self.vmin[2], p[2])
		))
		self.vmax = Vector((
				max(self.vmax[0], p[0]),
				max(self.vmax[1], p[1]),
				max(self.vmax[2], p[2])
		))
		
	def includeAll(self, points):
		"""Grows the local instance so that it includes every given point
		
		Parameters
		----------
		points : Vector[]
			A sequence of points to include in the local bounding box
		"""
		for p in points:
			self.includeOne(p);
	def center(self):
		"""Calculates a 3D center vector of the local instance
		
		Returns
		-------
		Vector
			The local center
		"""
		return (self.vmax + self.vmin) / 2.0;
	
	def size(self):
		"""Calculates a 3D size vector of the local instance
		
		Returns
		-------
		Vector
			The local diameter returned as a vector
		"""
		return (self.vmax - self.vmin);
	def maxSize(self):
		"""Calculates the maxmimum diameter along any primary axis.
		
		Returns
		-------
		float
			The maximum diameter of the local instance along the X, Y, and Z axes.
		"""
		size = self.size();
		return max(max(size.x, size.y),size.z);
	
class RadialSystem:
	"""A system with the given primary axis pointing along the Y axis, and Z as close to up as possible
	
	Used to convert points from their original tubular location to a flat approximation.
	"""
	def __init__(self, origin, primaryAxis, minY, maxY, baseRadius):
	
		primaryAxis.normalize();
		self.baseRadius = baseRadius;
		self.primaryAxis = primaryAxis.copy();
		self.origin = origin.copy();
		up = Vector((0,0,1));
		self.side = primaryAxis.cross(up).normalized();
		self.up = self.side.cross(primaryAxis).normalized();
		self.matrix = Matrix();
		self.matrix[0] = toVector4(self.side);
		self.matrix[1] = toVector4(self.primaryAxis);
		self.matrix[2] = toVector4(self.up);
		self.matrix[3] = toVector4(self.origin,1);
		self.matrix.transpose();
		#print (self.matrix);
		self.inverse = self.matrix.inverted();
		self.minY = minY;
		self.maxY = maxY;
		self.primaryAxis.freeze();
		self.origin.freeze();
		self.side.freeze();
		self.up.freeze();
		self.matrix.freeze();
		self.inverse.freeze();
		
	def pointAlongAxis(self,x):
		v = self.origin + self.primaryAxis * (self.minY + (self.maxY - self.minY) * x);
		v.resize_3d();
		return v;
		
	def getLoopCoord(self, world):
		"""Determines a linear identifier for the radial location of a world location
		
		Can be used to determine if two points should be connected by an edge of if that
		edge should instead connect to a point in the next loop (if any)
		
		
			
		Returns
		-------
		float
			A positional coordinate describing the loop location

		"""
		p = self.inverse @ world;
		return atan2(p.x, -p.z);	#(-pi,+pi] with 0=vertical down
		
	def loopCoordsConnect(self, a,b):
		"""Determines if two loop side values should be connect by an edge

		Parameters
		----------
		a : float
			First point loop side
		b : float
			Second point loop side
			
		Returns
		-------
		bool
			True if an edge should be created
		"""
		return abs(a-b) < pi;
		
	def unroll(self, world, repetition):
		"""Unrolls an assumed radial point to flattend coordinates
		
		The given world coordinates are transformed into the local coordinate space,
		then converted to polar coordinates along the X and Z directions.
		The polar coordinates (alpha, radius) are subsequently used to determine the unrolled X and Z coordinates.
		The world coordinates along the local axis direction remains unchanged.
		The resulting point is in world-space.
		
		Parameters
		----------
		world : Vector
			World coordinate of the point to unroll
		repetition: int
			The repetition index of the vertex to produce with 0 = first, 9 = tenth
			
		Returns
		-------
		Vector
			World-space coordinates after the unrolling
		"""
		p = self.inverse @ world;
		a = atan2(p.x, -p.z)/pi;	#(-1,+1] with 0=vertical down
		c = 2 * pi * self.baseRadius;
		x = a * c / 2 + repetition * c;
		
		return self.matrix @ Vector((x,p.y,Vector((p.x,p.z)).length - self.baseRadius));


class TubeMatch:
	"""Helper structure to detect the central axis of a drill tube and unroll the geometry.
	
	Must be created on an existing object which should be somewhat tubular and aligned roughly along the scene Y axis.
	Its bounding box center line, exactly along the world Y axis, becomes the first line to begin tube matching.
	This line must run through the tube for the most part or the result will not be correct.
	
	Attributes
	----------
	pointLine : Vector[]
		The last detected center line points with each xyz being the respective center point and w the radius
	obj : Blender object
		The source object
	bb : AABB
		The axis aligned bounding box of the source object
	resLine : int
		The number of samples to take along the previous center axis to produce the next generation of line points.
		Greater or equal to the number of entries in pointLine[]
	resCircle1 : int
		The number of radial rays to cast from each point during approx()
	resCircle2 : int
		The number of radial rays to cast from each point during refine()
	debugBoxSizes: float
		The size of debug boxes created during approx(True)
	"""
	pointLine = []

	obj = None;
	bb = AABB();
	resLine = 50;
	resCircle1 = 8;
	resCircle2 = 360;
	debugBoxSizes = 1;

	def __init__(self, obj):
		"""
		Parameters
		----------
		obj : Blender object
			The object to unroll
		"""
		if obj is None:
			print("Please select an object.")
			return

		self.obj = obj;
		self.bb.includeAll([obj.matrix_world @ v.co for v in obj.data.vertices]);

		# Calculate box dimensions based on bounding box
		self.box_center = self.bb.center();
		self.box_size = self.bb.size(); 

	def unroll(self, numRepeat=1):
		"""Unrolls the original geometry offset along the Y axis such that it appears next to the original.
		
		The unrolled object is optionally repeated to connect any spiral patterns.
		UV coordinates and materials are preserved, however complex material compositions might not appear as in the original.
		The quality of the unrolling depends on the current drill center detection quality.
		refine() has to be called at least once for a sufficiently precise radius to be present (approx() does not compute precise radii).
		
		Parameters
		----------
		numRepeat : int
			The number of times to repeat the unrolled object. Must be 1 or greater
		"""
		vertexes = [];
		polygons = [];
		mapToOldPolygons = [];
		space = self.regressPointLineToSpace();
		#bpy.ops.mesh.primitive_cube_add(location=Vector((0,0,0)), scale=Vector((1.0,100.0,1.0)));
		#bpy.context.object.matrix_world = space.matrix.copy();
		numVertexes = len(self.obj.data.vertices);
		print ('numVertexes ',numVertexes);
		for r in range(numRepeat):
			for v in self.obj.data.vertices:
				vtx = space.unroll(self.obj.matrix_world @ v.co,r);
				vtx.y += self.box_size.y*1.1;
				vertexes.append(vtx);
				#bpy.ops.mesh.primitive_cube_add(location=vtx, scale=Vector((1.0,1.0,1.0)));

				
		for p in self.obj.data.polygons:
			loopsides = [];
			for i in p.vertices:
				loopsides.append(space.getLoopCoord(self.obj.matrix_world @ self.obj.data.vertices[i].co));
			mn = min(loopsides);
			mx = max(loopsides);
			for r in range(numRepeat):
				polygon = [];
				complete = True;
				for i in range(len(p.vertices)):
					if (space.loopCoordsConnect(loopsides[i],mx)):
						polygon.append(p.vertices[i] + r * numVertexes);
					else:
						if (r+1 < numRepeat):
							polygon.append(p.vertices[i] + (r+1) * len(self.obj.data.vertices));
						else:
							complete = False;
				if complete:
					polygons.append(polygon);
					mapToOldPolygons.append(p);
				else:
					print('skipped ',p);
		
		#print(polygons);
		mesh = bpy.data.meshes.new('UnrolledMesh');
		newObj = bpy.data.objects.new('Unrolled',mesh);
		mesh.from_pydata(vertexes,[],polygons);
		
		
		for uvLayer in self.obj.data.uv_layers:
			newLayer = newObj.data.uv_layers.new(name=uvLayer.name);
			for iNewP in range(len(mesh.polygons)):
				newP = mesh.polygons[iNewP];
				oldP = mapToOldPolygons[iNewP];
				for lidx in range(min(newP.loop_total, oldP.loop_total)):
					olp = oldP.loop_start + lidx;
					nep = newP.loop_start + lidx;
					newLayer.data[nep].uv = uvLayer.data[olp].uv;
				
		for m in self.obj.data.materials:
			newObj.data.materials.append(m.copy());
		
		mesh.update()
		
		bpy.context.collection.objects.link(newObj)
		print(newObj);
						
			

	def regr(self, X):
		"""Computes a regression approximated axis and center point based on a set of points which may or may not form a line.
		
		Based on:
		https://stackoverflow.com/a/55604956
		
		Parameters
		----------
		X : numpy.array
			An array of points where each point is a 3 element array of floating point numbers
			
		Returns
		-------
		(ureduce,y,e,z)
			ureduce: resulting axis as a numpy float array [x,y,z]
			y: averaged center coordinates as a numpy float array [x,y,z]
			e: approximate locations of the input coordinates along the resulting line, as a numpy array of numpy float arrays [[x,y,z],...]
			z: factors of all input points along the resulting line
		"""

		y= numpy.average(X, axis=0)
		Xm = X-y
		u, s, v = numpy.linalg.svd((1./X.shape[0])*numpy.matmul(Xm.T,Xm))

		ureduce = u[:,0];
		# Extra Credit: Going back
		z= numpy.matmul(ureduce.T, Xm.T)
		#z = numpy.array([numpy.min(z),numpy.max(z)]);
		c = numpy.array([z*n for n in ureduce])
		d = numpy.array(y.tolist()*c.shape[1]).reshape(c.shape[1],-1).T
		e = (c+d).T
		return ureduce,y,e, z
	
	
	def regressPointLineToSpace(self):
		"""Computes an radial system based on the last determined axial center line.
		
		Points of the center line with a recorded radius <50% or >200% of the median radius are disregarded
		
		Returns
		-------
		RadialSystem
			The radial system approximated by the regression analysis of all previously determined center points
		"""
		
		median = statistics.median([v.w for v in self.pointLine]);
		points = [toVector3(v) for v in self.pointLine if v.w > median/2 and v.w < median * 2];
		(d, p, back, z) = self.regr(numpy.array([toArray(v) for v in points]));		
		return RadialSystem(
			origin=toVector(p),
			primaryAxis=toVector(d),
			minY=numpy.min(z),
			maxY=numpy.max(z),
			baseRadius=median);

	def refine(self, showCenterSpheres=False):
		"""Computes a more precise center line based on any previously determined center line using radial ray casts.
		
		If no previous center line exists, the bounding box center world Y axis is used.
		For each point along the last detected center line, rays are cast in every axis-orthogonal direction with a 1 degree delta.
		If all rays hit, the average of all intersections becomes the next center for this line sample.
		Their average distance from ray origin becomes its next radius.
			
		Before calling this function, approx() should be called at least once.
		Using ray casts without bounding box aggregation produces good results if the previous center line is already somewhat correct.
		When starting from scratch, it may take significantly more iterations to eventually be correct as scewed ray fans produces scewed 
		averaged intersection points and out-of-bounds radii.
		
		Parameters
		----------
		showCenterSpheres : bool
			If true, a chain of debug spheres is created along the determined center point sequence.
			Each sphere has the radius determined at that point.
		"""
		
		self.advance(self.resLine,self.resCircle2,False,False,showCenterSpheres, True);
			

	def advance(self, linearResolution, radialResolution, useBoundingBoxes, placeIntersectionObjects, placeCenterObjects, useRadius):
		if self.obj is None:
			return;
		obj = self.obj;
		if (len(self.pointLine) > 0):
			print('Creating space from existing');
			space = self.regressPointLineToSpace();
			self.pointLine = [];
		else:
			space = RadialSystem(
				origin=self.box_center,
				primaryAxis=Vector((0,1,0)),
				minY = self.bb.vmin.y - self.box_center.y,
				maxY = self.bb.vmax.y - self.box_center.y,
				baseRadius = 1 #don't know
				);

		mIn = obj.matrix_world.inverted();

		for y in range(linearResolution):
			fy = y / linearResolution;
			p0 = space.pointAlongAxis(fy);
			ref = mIn @ p0;
			if (useBoundingBoxes):
				bb = AABB();
			else:
				centerSum = Vector((0,0,0));
				radiusSum = 0;
			sampleCount = 0;
			complete = True;
			
			for r in range(radialResolution):
				fr = r / radialResolution;
				a = fr * 2.0 * pi;
				dir0 = Vector((cos(a),0,sin(a)));
				dir0 = rotate(space.matrix, dir0);
				dir = rotate(mIn, dir0);
				(hit, pos, normal,idx) = obj.ray_cast(ref, dir);

				if hit:
					sampleCount+=1;
					if (useBoundingBoxes):
						bb.includeOne(pos);
					else:
						centerSum += pos;
						radiusSum += (pos - ref).length;
					
					if (placeIntersectionObjects):
						p = obj.matrix_world @ pos;
						#p = p0 + dir0 * 100;
						bpy.ops.mesh.primitive_cube_add(location=p, scale=Vector((self.debugBoxSizes,self.debugBoxSizes,self.debugBoxSizes)));
					
					#centerSum += pos;
				else:
					#print('cycle not complete @'+str(y)+' '+str(ref)+' -> '+str(dir))
					complete = False;
					
			if complete:
				if useBoundingBoxes:
					p = obj.matrix_world @  bb.center();
					size = bb.maxSize();
				else:
					p = obj.matrix_world @  (centerSum / sampleCount);
					size = radiusSum / sampleCount;
					
				self.pointLine.append( toVector4(p, size));
				if placeCenterObjects:
					if useRadius:
						bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=4, location=p, radius=size);
					else:
						bpy.ops.mesh.primitive_cube_add(location=p, scale=Vector((1.0,1.0,1.0)));
	
	def approx(self, placeBoxes = False): 
		""" Computes an approximate center line based on the previously determined center line using radial ray casts.
		
		If no previous center line exists, the bounding box center world Y axis is used.
		For each point along the last detected center line, rays are cast in every axis-orthogonal direction with a 45 degree delta.
		If all rays hit, the center of the bounding box containing all intersections becomes the next center for this line sample.
		The maximum of the bounding box width/height/depth becomes its next radius.
			
		Using bounding boxes is not very precise but it does not suffer scewing as much even if the current center is way off the center of the tube.
		Calling this function several times may improve the approximation but probably won't much.
			
		Parameters
		----------
		placeBoxes : bool
			If true, debug boxes are created at each ray intersection
		"""
		self.advance(self.resLine,self.resCircle1,True, placeBoxes, False, False);
		

tm = TubeMatch(bpy.context.object);
tm.approx();
#tm.approx();
tm.refine();
#tm.refine();
tm.unroll(3);

