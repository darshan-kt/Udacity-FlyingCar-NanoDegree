from shapely.geometry import Polygon, Point
#Shapely is a python package used to define polygon object.


coords = [(0,0), (1,0), (1,1), (0,1)]   #x-y co-oridiantes 
poly = Polygon(coords)         #Create polygon object using co-orinates and named as poly

print(poly)
#Attribute the created polygon object
print(poly.area)
print(poly.boundary)
print(poly.bounds)
print(poly.length)

#Check test the given point lies within the polygon or not
p1 = Point(0.5, 0.5)
p2 = Point(1.5, 1.5)
print(poly.contains(p1))   #lies within the shape
print(poly.contains(p2))   #not lieing within the shape



#3D polygon
coords = [(0,0,1), (1,0,1), (1,1,1), (0,1,1)]   #x-y-z co-oridiantes 
poly3D = Polygon(coords)         #Create polygon object using co-orinates and named as poly
print(poly3D)