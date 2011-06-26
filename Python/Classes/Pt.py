#!/usr/bin/python

class Pt(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
        
    def __str__(self):
        return '(%0.4f, %0.4f, %0.4f)' % (self.x, self.y, self.z)

    def __repr__(self):
        return 'Pt(%f, %f, %f)' % (self.x, self.y, self.z)

    def __add__(self, other):
        return Pt(self.x+other.x, self.y+other.y, self.z+other.z)

    def __sub__(self, other):
        return Pt(self.x-other.x, self.y-other.y, self.z-other.z)
    
    def __mul__(self, f):
        return Pt(self.x*f, self.y*f, self.z*f)

    def dist(self, other):
        p = self-other
        return (p.x**2 + p.y**2 + p.z**2)**0.5

    def toSpherical(self):
        r = mag(self)
        theta = atan2(sqrt(self.x**2+self.y**2), self.z)
        phi = atan2(self.y, self.x)
        return SphericalPt(r, theta, phi)


class SphericalPt(object):
    def __init__(self, r, theta, phi):
        # radial coordinate, zenith angle, azimuth angle
        self.r = r
        self.theta = theta
        self.phi = phi

    def __str__(self):
        return '(%0.4f, %0.4f, %0.4f)' % (self.r, self.theta, self.phi)

    def __repr__(self):
        return 'SphericalPt(%f, %f, %f)' % (self.r, self.theta, self.phi)

    def toCartesian(self):
        x = self.r*cos(self.phi)*sin(self.theta)
        y = self.r*sin(self.phi)*sin(self.theta)
        z = self.r*cos(self.theta)
        return Pt(x,y,z)
