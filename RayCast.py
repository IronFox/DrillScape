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
	
def uv_from_vert_first(uv_layer, v):
    for l in v.link_loops:
        uv_data = l[uv_layer]
        return uv_data.uv
    return None
	
def setAllUv(uvLayer, v, uv):
	for l in v.link_loops:
		l[uv_layer].uv = uv;


class AABB:
	
	vmin = Vector((float('inf'), float('inf'), float('inf')))
	vmax = Vector((float('-inf'), float('-inf'), float('-inf')))
	
	
	def includeOne(self, p):
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
		for p in points:
			self.includeOne(p);
	def center(self):
		return (self.vmax + self.vmin) / 2.0;
	
	def size(self):
		return (self.vmax - self.vmin);
	def maxSize(self):
		size = self.size();
		return max(max(size.x, size.y),size.z);
	
'''
Space with the given primary axis pointing along the Y axis, and Z as close to up as possible
'''
class AxialSpace:
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
		
	def getLoopSide(self, world):
		p = self.inverse @ world;
		return atan2(p.x, -p.z);	#(-pi,+pi] with 0=vertical down
	def loopSidesConnected(self, a,b):
		return abs(a-b) < pi/2;
		
	def unroll(self, world, repetition):
		p = self.inverse @ world;
		a = atan2(p.x, -p.z)/pi;	#(-1,+1] with 0=vertical down
		c = 2 * pi * self.baseRadius;
		x = a * c / 2 + repetition * c;
		
		return self.matrix @ Vector((x,p.y,Vector((p.x,p.z)).length - self.baseRadius));


class TubeMatch:
	pointLine = []

	obj = None;
	bb = AABB();
	resLine = 50;
	resCircle1 = 8;
	resCircle2 = 64;
	

	def __init__(self, obj):
		if obj is None:
			print("Please select an object.")
			return

		self.obj = obj;
		self.bb.includeAll([obj.matrix_world @ v.co for v in obj.data.vertices]);

		# Calculate box dimensions based on bounding box
		self.box_center = self.bb.center();
		self.box_size = self.bb.size(); 


	def unroll(self, numRepeat=1):
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
				loopsides.append(space.getLoopSide(self.obj.matrix_world @ self.obj.data.vertices[i].co));
			mn = min(loopsides);
			mx = max(loopsides);
			for r in range(numRepeat):
				polygon = [];
				complete = True;
				for i in range(len(p.vertices)):
					if (space.loopSidesConnected(loopsides[i],mx)):
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
		median = statistics.median([v.w for v in self.pointLine]);
		points = [toVector3(v) for v in self.pointLine if v.w > median/2 and v.w < median * 2];
		(d, p, back, z) = self.regr(numpy.array([toArray(v) for v in points]));		
		return AxialSpace(
			origin=toVector(p),
			primaryAxis=toVector(d),
			minY=numpy.min(z),
			maxY=numpy.max(z),
			baseRadius=median);

	def stage2(self):
		print(self.pointLine);
		(self.regv, self.regp, back) = self.regr(numpy.array([toArray(v) for v in self.pointLine]));
		self.regv = toVector(self.regv);
		self.regp = toVector(self.regp);
		print (self.regv);
		print (self.regp);
		
		
		#for p in self.pointLine:
		#	bpy.ops.mesh.primitive_cube_add(location=p, scale=Vector((1,1,1)));
#		for p in [toVector(v) for v in back]:
#			bpy.ops.mesh.primitive_cube_add(location=p, scale=Vector((1,1,1)));
#		
		for x in range(20):
			fx = (x / 20  -0.5) * self.box_size.y;
			p = self.regp + self.regv * fx;
			bpy.ops.mesh.primitive_cube_add(location=p, scale=Vector((1,1,1)));


	def refine(self, showCenterSpheres=False):
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
			space = AxialSpace(
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
						bpy.ops.mesh.primitive_cube_add(location=p, scale=Vector((1,1,1)));
					
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
		self.advance(self.resLine,self.resCircle1,True, placeBoxes, False, False);
		

tb = TubeMatch(bpy.context.object);
tb.approx();
#tb.approx();
tb.refine();
#tb.refine();
tb.unroll(3);

#tb.stage2();

