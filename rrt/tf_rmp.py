import numpy
import math

def translate(tx,ty,tz):
    """ Returns a translation matrix for specific distances
    Args:
        dx (double): distance to translate along x-axis
        dy (double): distance to translate along y-axis
        dz (double): distance to translate along z-axis
    Returns:
        numpy.matrix: translation matrix
    """
    result = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')

    result[0,3] = tx
    result[1,3] = ty
    result[2,3] = tz

    return result


def rotateX(rad):
    """ Returns a rotation matrix for rotating about the x-axis
    Args:
        rad (double): radian amount to rotate
    Returns:
        numpy.matrix: rotation matrix
    """
    result = numpy.matrix('0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 1.0')

    result[0,0] = 1

    result[1,1] = math.cos(rad)
    result[1,2] = -math.sin(rad)
    result[2,1] = math.sin(rad)
    result[2,2] = math.cos(rad)

    return result


def rotateY(rad):
    """ Returns a rotation matrix for rotating about the y-axis
    Args:
        rad (double): radian amount to rotate
    Returns:
        numpy.matrix: rotation matrix
    """
    result = numpy.matrix('0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 1.0')

    result[1,1] = 1

    result[0,0] = math.cos(rad)
    result[2,0] = -math.sin(rad)
    result[0,2] = math.sin(rad)
    result[2,2] = math.cos(rad)

    return result


def rotateZ(rad):
    """ Returns a rotation matrix for rotating about the z-axis
    Args:
        rad (double): radian amount to rotate
    Returns:
        numpy.matrix: rotation matrix
    """
    result = numpy.matrix('0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 0.0; 0.0 0.0 0.0 1.0')

    result[2,2] = 1

    result[0,0] = math.cos(rad)
    result[0,1] = -math.sin(rad)
    result[1,0] = math.sin(rad)
    result[1,1] = math.cos(rad)

    return result

def rotate_zyx(rx,ry,rz):
    """ Returns a rotation matrix for rotating about a general axis with a euler angle product of z*y*x
    Args:
        rot_z (double): radian amount to rotate about the z-axis
        rot_y (double): radian amount to rotate about the y-axis
        rot_x (double): radian amount to rotate about the x-axis
    Returns:
        numpy.matrix: rotation matrix
    """
    rot_x = rotateX(rx)
    rot_y = rotateY(ry)
    rot_z = rotateZ(rz)

    result = rot_z * rot_y * rot_x
    
    return result


def transform(tx,ty,tz,rx,ry,rz):
    """ Returns a transformation matrix that incorporates both translation and rotation
    The matrix should reflect rotating z-y-x and then translating.
    Args:
        dx (double): distance to translate along x-axis
        dy (double): distance to translate along y-axis
        dz (double): distance to translate along z-axis
        rot_x (double): radian amount to rotate about the x-axis
        rot_y (double): radian amount to rotate about the y-axis
        rot_z (double): radian amount to rotate about the z-axis
    Returns:
        numpy.matrix: transformation matrix
    """
    tr = translate(tx,ty,tz)
    rot = rotate_zyx(rx,ry,rz)
    
    return tr*rot