import math


class Quaternion:
    def __init__(self, w=1, x=0, y=0, z=0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "w: " + str(self.w) + " x: " + str(self.x) + " y: " +  str(self.y) + " z: " + str(self.z)


    def get_product(self, q):
        """Multiply a Quaternion by another
        :param q: Quaternion -- quaternion to multiply by
        :return: Quaternion
        """
        # Quaternion multiplication is defined by:
        #     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
        #     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
        #     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
        #     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
        return Quaternion(
            self.w * q.w - self.x * q.x - self.y * q.y - self.z * q.z,  # new w
            self.w * q.x + self.x * q.w + self.y * q.z - self.z * q.y,  # new x
            self.w * q.y - self.x * q.z + self.y * q.w + self.z * q.x,  # new y
            self.w * q.z + self.x * q.y - self.y * q.x + self.z * q.w  # new z
        )

    def get_conjugate(self):
        """Get the conjugate of this Quaternion
        :return: Quaternion
        """
        return Quaternion(self.w, -1.0 * self.x, -1.0 * self.y, -1.0 * self.z)

    def get_magnitude(self):
        """Get magnitude of this quaternion
        :return: float
        """
        return math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        """Normalize this Quaternion (i.e. divide by magnitude)
        :return:
        """
        m = self.get_magnitude()
        self.w /= m
        self.x /= m
        self.y /= m
        self.z /= m

    def get_normalized(self):
        """Get normalized version of this Quaternion
        :return: Quaternion
        """
        r = Quaternion(self.w, self.x, self.y, self.z)
        r.normalize()
        return r

    def get_euler_angles(self):
        phi = math.atan2(2.0 * (self.w * self.x + self.y * self.z), 1.0 - 2.0 * (self.x * self.x + self.y * self.y))
        theta = math.asin(2.0 * (self.w * self.y - self.z * self.x))
        psi = math.atan2(2.0 * (self.w * self.z + self.x * self.y), 1.0 - 2.0 * (self.y * self.y + self.z * self.z))
        return phi, theta, psi

    def from_accel(self, a):
        if a.z >= 0:
            q = Quaternion(
                math.sqrt((a.z + 1)/2),
                - a.y / math.sqrt(2.0 * (a.z + 1)),
                a.x / math.sqrt(2.0 * (a.z + 1)),
                0
            )
        else:
            q = Quaternion(
                - a.y / math.sqrt(2.0 * (1 - a.z)),
                math.sqrt((1 - a.z) / 2),
                0,
                a.x / math.sqrt(2.0 * (1 - a.z))
            )
        return q

    @property
    def conjugate(self):
        """Conjugate of this Quaternion
        :return:
        """
        return self.get_conjugate()

    @property
    def magnitude(self):
        """Magnitude of this Quaternion
        :return:
        """
        return self.get_magnitude()

    @property
    def normalized(self):
        """Normalized version of this Quaternion
        :return:
        """
        return self.get_normalized()


class Vector:
    def __init__(self, x, y, z):
        """Create a 3D Vector
        :param x: float
        :param y: float
        :param z: float
        """
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "x: " + str(self.x) + " y: " +  str(self.y) + " z: " + str(self.z)

    def get_magnitude(self):
        """Get the magnitude of this Vector
        :return: float
        """
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        """Normalize this Vector
        :return:
        """
        m = self.get_magnitude()
        self.x /= m
        self.y /= m
        self.z /= m

    def get_normalized(self):
        """Get normalized version of this Vector
        :return: Vector
        """
        r = Vector(self.x, self.y, self.z)
        r.normalize()
        return r

    def rotate(self, q):
        """Rotate Vector using a Quaternion
         P_out = q * P_in * conj(q)
         - P_out is the output vector
         - q is the orientation quaternion
         - P_in is the input vector (a*aReal)
         - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
        :param q: Quaternion
        :return:
        """
        p = Quaternion(0, self.x, self.y, self.z)

        # http://www.cprogramming.com/tutorial/3d/quaternions.html
        # http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
        # http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
        # ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

        p = q.get_product(p)
        p = p.get_product(q.conjugate)
        # p is now [0, x', y', z']
        self.x = p.x
        self.y = p.y
        self.z = p.z

    def get_rotated(self, q):
        """Get rotated version of this Vector
        :param q: Quaternion
        :return: Vector
        """
        r = Vector(self.x, self.y, self.z)
        r.rotate(q)
        return r

    @property
    def magnitude(self):
        """Get magnitude of this Vector
        :return: float
        """
        return self.get_magnitude()

    @property
    def normalized(self):
        """Get normalized version of this Vector
        :return: Vector
        """
        return self.get_normalized()
