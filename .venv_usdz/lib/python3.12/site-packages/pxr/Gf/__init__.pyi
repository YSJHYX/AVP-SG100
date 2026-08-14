from __future__ import annotations
import pxr.Gf._gf
import typing
import Boost.Python
import pxr.Gf

__all__ = [
    "Abs",
    "Absf",
    "ApplyGamma",
    "BBox3d",
    "Camera",
    "Ceil",
    "Ceilf",
    "Clamp",
    "Clampf",
    "Color",
    "ColorSpace",
    "ColorSpaceNames",
    "CompDiv",
    "CompMult",
    "ConvertDisplayToLinear",
    "ConvertLinearToDisplay",
    "Cross",
    "DegreesToRadians",
    "Dot",
    "DualQuatd",
    "DualQuatf",
    "DualQuath",
    "Exp",
    "Expf",
    "FindClosestPoints",
    "FitPlaneToPoints",
    "Floor",
    "Floorf",
    "Frustum",
    "GetComplement",
    "GetDisplayGamma",
    "GetHomogenized",
    "GetLength",
    "GetNormalized",
    "GetProjection",
    "HomogeneousCross",
    "Interval",
    "IsClose",
    "Lerp",
    "Lerpf",
    "Line",
    "LineSeg",
    "Log",
    "Logf",
    "MIN_ORTHO_TOLERANCE",
    "MIN_VECTOR_LENGTH",
    "Matrix2d",
    "Matrix2f",
    "Matrix3d",
    "Matrix3f",
    "Matrix4d",
    "Matrix4f",
    "Max",
    "Min",
    "Mod",
    "Modf",
    "MultiInterval",
    "Normalize",
    "Plane",
    "Pow",
    "Powf",
    "Project",
    "Quatd",
    "Quaternion",
    "Quatf",
    "Quath",
    "RadiansToDegrees",
    "Range1d",
    "Range1f",
    "Range2d",
    "Range2f",
    "Range3d",
    "Range3f",
    "Ray",
    "Rect2i",
    "Rotation",
    "Round",
    "Roundf",
    "Sgn",
    "Size2",
    "Size3",
    "Slerp",
    "SmoothStep",
    "Sqr",
    "Sqrt",
    "Sqrtf",
    "Transform",
    "Vec2d",
    "Vec2f",
    "Vec2h",
    "Vec2i",
    "Vec3d",
    "Vec3f",
    "Vec3h",
    "Vec3i",
    "Vec4d",
    "Vec4f",
    "Vec4h",
    "Vec4i"
]


class BBox3d(Boost.Python.instance):
    """
    Arbitrarily oriented 3D bounding box
    """
    @staticmethod
    def Combine(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeAlignedBox(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeAlignedRange(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeCentroid(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBox(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverseMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRange(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVolume(*args, **kwargs) -> None: ...
    @staticmethod
    def HasZeroAreaPrimitives(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @staticmethod
    def SetHasZeroAreaPrimitives(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRange(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def box(self) -> None:
        """
        :type: None
        """
    @property
    def hasZeroAreaPrimitives(self) -> None:
        """
        :type: None
        """
    @property
    def matrix(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 336
    pass
class Camera(Boost.Python.instance):
    class FOVDirection(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'Camera'
        allValues: tuple # value = (Gf.Camera.FOVHorizontal, Gf.Camera.FOVVertical)
        pass
    class Projection(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'Camera'
        allValues: tuple # value = (Gf.Camera.Perspective, Gf.Camera.Orthographic)
        pass
    @staticmethod
    def GetFieldOfView(*args, **kwargs) -> None: ...
    @staticmethod
    def SetFromViewAndProjectionMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def SetOrthographicFromAspectRatioAndSize(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPerspectiveFromAspectRatioAndFieldOfView(*args, **kwargs) -> None: ...
    @property
    def aspectRatio(self) -> None:
        """
        :type: None
        """
    @property
    def clippingPlanes(self) -> None:
        """
        :type: None
        """
    @property
    def clippingRange(self) -> None:
        """
        :type: None
        """
    @property
    def fStop(self) -> None:
        """
        :type: None
        """
    @property
    def focalLength(self) -> None:
        """
        :type: None
        """
    @property
    def focusDistance(self) -> None:
        """
        :type: None
        """
    @property
    def frustum(self) -> None:
        """
        :type: None
        """
    @property
    def horizontalAperture(self) -> None:
        """
        :type: None
        """
    @property
    def horizontalApertureOffset(self) -> None:
        """
        :type: None
        """
    @property
    def horizontalFieldOfView(self) -> None:
        """
        :type: None
        """
    @property
    def projection(self) -> None:
        """
        :type: None
        """
    @property
    def transform(self) -> None:
        """
        :type: None
        """
    @property
    def verticalAperture(self) -> None:
        """
        :type: None
        """
    @property
    def verticalApertureOffset(self) -> None:
        """
        :type: None
        """
    @property
    def verticalFieldOfView(self) -> None:
        """
        :type: None
        """
    APERTURE_UNIT = 0.1
    DEFAULT_HORIZONTAL_APERTURE = 20.955
    DEFAULT_VERTICAL_APERTURE = 15.290799999999999
    FOCAL_LENGTH_UNIT = 0.1
    FOVHorizontal: pxr.Gf.FOVDirection # value = Gf.Camera.FOVHorizontal
    FOVVertical: pxr.Gf.FOVDirection # value = Gf.Camera.FOVVertical
    Orthographic: pxr.Gf.Projection # value = Gf.Camera.Orthographic
    Perspective: pxr.Gf.Projection # value = Gf.Camera.Perspective
    __instance_size__ = 216
    pass
class Color(Boost.Python.instance):
    """
    A class representing a color, supporting different color spaces.
    """
    @staticmethod
    def GetColorSpace(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRGB(*args, **kwargs) -> None: ...
    @staticmethod
    def SetFromPlanckianLocus(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class ColorSpace(Boost.Python.instance):
    @staticmethod
    def Convert(*args, **kwargs) -> None: ...
    @staticmethod
    def ConvertRGBASpan(*args, **kwargs) -> None: ...
    @staticmethod
    def ConvertRGBSpan(*args, **kwargs) -> None: ...
    @staticmethod
    def GetGamma(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLinearBias(*args, **kwargs) -> None: ...
    @staticmethod
    def GetName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrimariesAndWhitePoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRGBToXYZ(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTransferFunctionParams(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValid(*args, **kwargs) -> None: ...
    __instance_size__ = 40
    pass
class ColorSpaceNames(Boost.Python.instance):
    CIEXYZ = 'lin_ciexyzd65_scene'
    Data = 'data'
    G18Rec709 = 'g18_rec709_scene'
    G22AP1 = 'g22_ap1_scene'
    G22AdobeRGB = 'g22_adobergb_scene'
    G22Rec709 = 'g22_rec709_scene'
    Identity = 'identity'
    LinearAP0 = 'lin_ap0_scene'
    LinearAP1 = 'lin_ap1_scene'
    LinearAdobeRGB = 'lin_adobergb_scene'
    LinearCIEXYZD65 = 'lin_ciexyzd65_scene'
    LinearDisplayP3 = 'lin_p3d65_scene'
    LinearP3D65 = 'lin_p3d65_scene'
    LinearRec2020 = 'lin_rec2020_scene'
    LinearRec709 = 'lin_rec709_scene'
    Raw = 'raw'
    SRGBAP1 = 'srgb_ap1_scene'
    SRGBP3D65 = 'srgb_p3d65_scene'
    SRGBRec709 = 'srgb_rec709_scene'
    Unknown = 'unknown'
    pass
class DualQuatd(Boost.Python.instance):
    @staticmethod
    def GetConjugate(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDual(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDual(*args, **kwargs) -> None: ...
    @staticmethod
    def SetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def dual(self) -> None:
        """
        :type: None
        """
    @property
    def real(self) -> None:
        """
        :type: None
        """
    pass
class DualQuatf(Boost.Python.instance):
    @staticmethod
    def GetConjugate(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDual(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDual(*args, **kwargs) -> None: ...
    @staticmethod
    def SetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def dual(self) -> None:
        """
        :type: None
        """
    @property
    def real(self) -> None:
        """
        :type: None
        """
    pass
class DualQuath(Boost.Python.instance):
    @staticmethod
    def GetConjugate(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDual(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDual(*args, **kwargs) -> None: ...
    @staticmethod
    def SetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def dual(self) -> None:
        """
        :type: None
        """
    @property
    def real(self) -> None:
        """
        :type: None
        """
    pass
class Frustum(Boost.Python.instance):
    """
    Basic view frustum
    """
    class ProjectionType(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'Frustum'
        allValues: tuple # value = (Gf.Frustum.Orthographic, Gf.Frustum.Perspective)
        pass
    @staticmethod
    def ComputeAspectRatio(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeCorners(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeCornersAtDistance(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeLookAtPoint(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeNarrowedFrustum(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputePickRay(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeProjectionMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeUpVector(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeViewDirection(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeViewFrame(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeViewInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeViewMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def FitToSphere(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFOV(*args, **kwargs) -> None: 
        """
        Returns the horizontal fov of the frustum. The fov of the
        frustum is not necessarily the same value as displayed in
        the viewer. The displayed fov is a function of the focal
        length or FOV avar. The frustum's fov may be different due
        to things like lens breathing.

        If the frustum is not of type GfFrustum::Perspective, the
        returned FOV will be 0.0.
        """
    @staticmethod
    def GetNearFar(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOrthographic(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPerspective(*args, **kwargs) -> None: 
        """
        Returns the current perspective frustum values suitable
        for use by SetPerspective.  If the current frustum is a
        perspective projection, the return value is a tuple of
        fieldOfView, aspectRatio, nearDistance, farDistance).
        If the current frustum is not perspective, the return
        value is None.
        """
    @staticmethod
    def GetPosition(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjectionType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetReferencePlaneDepth(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetViewDistance(*args, **kwargs) -> None: ...
    @staticmethod
    def GetWindow(*args, **kwargs) -> None: ...
    @staticmethod
    def Intersects(*args, **kwargs) -> None: ...
    @staticmethod
    def IntersectsViewVolume(*args, **kwargs) -> None: ...
    @staticmethod
    def SetNearFar(*args, **kwargs) -> None: ...
    @staticmethod
    def SetOrthographic(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPerspective(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPosition(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPositionAndRotationFromMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def SetProjectionType(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def SetViewDistance(*args, **kwargs) -> None: ...
    @staticmethod
    def SetWindow(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def nearFar(self) -> None:
        """
        :type: None
        """
    @property
    def position(self) -> None:
        """
        :type: None
        """
    @property
    def projectionType(self) -> None:
        """
        :type: None
        """
    @property
    def rotation(self) -> None:
        """
        :type: None
        """
    @property
    def viewDistance(self) -> None:
        """
        :type: None
        """
    @property
    def window(self) -> None:
        """
        :type: None
        """
    Orthographic: pxr.Gf.ProjectionType # value = Gf.Frustum.Orthographic
    Perspective: pxr.Gf.ProjectionType # value = Gf.Frustum.Perspective
    __instance_size__ = 152
    pass
class Interval(Boost.Python.instance):
    """
    Basic mathematical interval class
    """
    @staticmethod
    def Contains(*args, **kwargs) -> None: 
        """
        Returns true if x is inside the interval.

        Returns true if x is inside the interval.
        """
    @staticmethod
    def GetFullInterval(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMax(*args, **kwargs) -> None: 
        """
        Get the maximum value.
        """
    @staticmethod
    def GetMin(*args, **kwargs) -> None: 
        """
        Get the minimum value.
        """
    @staticmethod
    def GetSize(*args, **kwargs) -> None: 
        """
        The width of the interval
        """
    @staticmethod
    def In(*args, **kwargs) -> None: 
        """
        Returns true if x is inside the interval.
        """
    @staticmethod
    def Intersects(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: 
        """
        True if the interval is empty.
        """
    @staticmethod
    def IsFinite(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMaxClosed(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMaxFinite(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMaxOpen(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMinClosed(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMinFinite(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMinOpen(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMax(*args, **kwargs) -> None: 
        """
        Set the maximum value.

        Set the maximum value and boundary condition.
        """
    @staticmethod
    def SetMin(*args, **kwargs) -> None: 
        """
        Set the minimum value.

        Set the minimum value and boundary condition.
        """
    @property
    def finite(self) -> None:
        """
        :type: None
        """
    @property
    def isEmpty(self) -> None:
        """
        True if the interval is empty.

        :type: None
        """
    @property
    def max(self) -> None:
        """
        The maximum value.

        :type: None
        """
    @property
    def maxClosed(self) -> None:
        """
        :type: None
        """
    @property
    def maxFinite(self) -> None:
        """
        :type: None
        """
    @property
    def maxOpen(self) -> None:
        """
        :type: None
        """
    @property
    def min(self) -> None:
        """
        The minimum value.

        :type: None
        """
    @property
    def minClosed(self) -> None:
        """
        :type: None
        """
    @property
    def minFinite(self) -> None:
        """
        :type: None
        """
    @property
    def minOpen(self) -> None:
        """
        :type: None
        """
    @property
    def size(self) -> None:
        """
        The width of the interval.

        :type: None
        """
    __instance_size__ = 56
    pass
class Line(Boost.Python.instance):
    """
    Line class
    """
    @staticmethod
    def FindClosestPoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDirection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPoint(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @property
    def direction(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 72
    pass
class LineSeg(Boost.Python.instance):
    """
    Line segment class
    """
    @staticmethod
    def FindClosestPoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDirection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPoint(*args, **kwargs) -> None: ...
    @property
    def direction(self) -> None:
        """
        :type: None
        """
    @property
    def length(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 80
    pass
class Matrix2d(Boost.Python.instance):
    @staticmethod
    def GetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDeterminant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranspose(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @staticmethod
    def SetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDiagonal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def SetZero(*args, **kwargs) -> None: ...
    __safe_for_unpickling__ = True
    dimension = (2, 2)
    pass
class Matrix2f(Boost.Python.instance):
    @staticmethod
    def GetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDeterminant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranspose(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @staticmethod
    def SetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDiagonal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def SetZero(*args, **kwargs) -> None: ...
    __safe_for_unpickling__ = True
    dimension = (2, 2)
    pass
class Matrix3d(Boost.Python.instance):
    @staticmethod
    def ExtractRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDeterminant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHandedness(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOrthonormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranspose(*args, **kwargs) -> None: ...
    @staticmethod
    def IsLeftHanded(*args, **kwargs) -> None: ...
    @staticmethod
    def IsRightHanded(*args, **kwargs) -> None: ...
    @staticmethod
    def Orthonormalize(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @staticmethod
    def SetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDiagonal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotate(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def SetScale(*args, **kwargs) -> None: ...
    @staticmethod
    def SetZero(*args, **kwargs) -> None: ...
    __safe_for_unpickling__ = True
    dimension = (3, 3)
    pass
class Matrix3f(Boost.Python.instance):
    @staticmethod
    def ExtractRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDeterminant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHandedness(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOrthonormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranspose(*args, **kwargs) -> None: ...
    @staticmethod
    def IsLeftHanded(*args, **kwargs) -> None: ...
    @staticmethod
    def IsRightHanded(*args, **kwargs) -> None: ...
    @staticmethod
    def Orthonormalize(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @staticmethod
    def SetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDiagonal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotate(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def SetScale(*args, **kwargs) -> None: ...
    @staticmethod
    def SetZero(*args, **kwargs) -> None: ...
    __safe_for_unpickling__ = True
    dimension = (3, 3)
    pass
class Matrix4d(Boost.Python.instance):
    @staticmethod
    def ExtractRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def ExtractRotationMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def ExtractRotationQuat(*args, **kwargs) -> None: ...
    @staticmethod
    def ExtractTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def Factor(*args, **kwargs) -> None: ...
    @staticmethod
    def GetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDeterminant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDeterminant3(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHandedness(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOrthonormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRow3(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranspose(*args, **kwargs) -> None: ...
    @staticmethod
    def HasOrthogonalRows3(*args, **kwargs) -> None: ...
    @staticmethod
    def IsLeftHanded(*args, **kwargs) -> None: ...
    @staticmethod
    def IsRightHanded(*args, **kwargs) -> None: ...
    @staticmethod
    def Orthonormalize(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveScaleShear(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @staticmethod
    def SetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDiagonal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def SetLookAt(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotate(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotateOnly(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRow3(*args, **kwargs) -> None: ...
    @staticmethod
    def SetScale(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTransform(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTranslate(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTranslateOnly(*args, **kwargs) -> None: ...
    @staticmethod
    def SetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @staticmethod
    def TransformAffine(*args, **kwargs) -> None: ...
    @staticmethod
    def TransformDir(*args, **kwargs) -> None: ...
    __safe_for_unpickling__ = True
    dimension = (4, 4)
    pass
class Matrix4f(Boost.Python.instance):
    @staticmethod
    def ExtractRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def ExtractRotationMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def ExtractRotationQuat(*args, **kwargs) -> None: ...
    @staticmethod
    def ExtractTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def Factor(*args, **kwargs) -> None: ...
    @staticmethod
    def GetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDeterminant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDeterminant3(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHandedness(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOrthonormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRow3(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranspose(*args, **kwargs) -> None: ...
    @staticmethod
    def HasOrthogonalRows3(*args, **kwargs) -> None: ...
    @staticmethod
    def IsLeftHanded(*args, **kwargs) -> None: ...
    @staticmethod
    def IsRightHanded(*args, **kwargs) -> None: ...
    @staticmethod
    def Orthonormalize(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveScaleShear(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @staticmethod
    def SetColumn(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDiagonal(*args, **kwargs) -> None: ...
    @staticmethod
    def SetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def SetLookAt(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotate(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotateOnly(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRow(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRow3(*args, **kwargs) -> None: ...
    @staticmethod
    def SetScale(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTransform(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTranslate(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTranslateOnly(*args, **kwargs) -> None: ...
    @staticmethod
    def SetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @staticmethod
    def TransformAffine(*args, **kwargs) -> None: ...
    @staticmethod
    def TransformDir(*args, **kwargs) -> None: ...
    __safe_for_unpickling__ = True
    dimension = (4, 4)
    pass
class MultiInterval(Boost.Python.instance):
    @staticmethod
    def Add(*args, **kwargs) -> None: ...
    @staticmethod
    def ArithmeticAdd(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def Contains(*args, **kwargs) -> None: 
        """
        Returns true if x is inside the multi-interval.

        Returns true if x is inside the multi-interval.

        Returns true if x is inside the multi-interval.
        """
    @staticmethod
    def GetBounds(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetContainingInterval(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFullInterval(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNextNonContainingInterval(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPriorNonContainingInterval(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def Intersect(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def Remove(*args, **kwargs) -> None: ...
    @property
    def bounds(self) -> None:
        """
        :type: None
        """
    @property
    def isEmpty(self) -> None:
        """
        :type: None
        """
    @property
    def size(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 72
    pass
class Plane(Boost.Python.instance):
    @staticmethod
    def GetDistance(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDistanceFromOrigin(*args, **kwargs) -> None: ...
    @staticmethod
    def GetEquation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormal(*args, **kwargs) -> None: ...
    @staticmethod
    def IntersectsPositiveHalfSpace(*args, **kwargs) -> None: ...
    @staticmethod
    def Project(*args, **kwargs) -> None: ...
    @staticmethod
    def Reorient(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def distanceFromOrigin(self) -> None:
        """
        :type: None
        """
    @property
    def normal(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 56
    pass
class Quatd(Boost.Python.instance):
    @staticmethod
    def GetConjugate(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def GetImaginary(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def GetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def SetImaginary(*args, **kwargs) -> None: ...
    @staticmethod
    def SetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def imaginary(self) -> None:
        """
        :type: None
        """
    @property
    def real(self) -> None:
        """
        :type: None
        """
    pass
class Quaternion(Boost.Python.instance):
    """
    Quaternion class
    """
    @staticmethod
    def GetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def GetImaginary(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def GetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @property
    def imaginary(self) -> None:
        """
        :type: None
        """
    @property
    def real(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 56
    pass
class Quatf(Boost.Python.instance):
    @staticmethod
    def GetConjugate(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def GetImaginary(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def GetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def SetImaginary(*args, **kwargs) -> None: ...
    @staticmethod
    def SetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def imaginary(self) -> None:
        """
        :type: None
        """
    @property
    def real(self) -> None:
        """
        :type: None
        """
    pass
class Quath(Boost.Python.instance):
    @staticmethod
    def GetConjugate(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def GetImaginary(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def GetZero(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def SetImaginary(*args, **kwargs) -> None: ...
    @staticmethod
    def SetReal(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def imaginary(self) -> None:
        """
        :type: None
        """
    @property
    def real(self) -> None:
        """
        :type: None
        """
    pass
class Range1d(Boost.Python.instance):
    @staticmethod
    def Contains(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDistanceSquared(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIntersection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMidpoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUnion(*args, **kwargs) -> None: ...
    @staticmethod
    def IntersectWith(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def UnionWith(*args, **kwargs) -> None: ...
    @property
    def max(self) -> None:
        """
        :type: None
        """
    @property
    def min(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    dimension = 1
    pass
class Range1f(Boost.Python.instance):
    @staticmethod
    def Contains(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDistanceSquared(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIntersection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMidpoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUnion(*args, **kwargs) -> None: ...
    @staticmethod
    def IntersectWith(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def UnionWith(*args, **kwargs) -> None: ...
    @property
    def max(self) -> None:
        """
        :type: None
        """
    @property
    def min(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 32
    dimension = 1
    pass
class Range2d(Boost.Python.instance):
    @staticmethod
    def Contains(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCorner(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDistanceSquared(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIntersection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMidpoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def GetQuadrant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUnion(*args, **kwargs) -> None: ...
    @staticmethod
    def IntersectWith(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def UnionWith(*args, **kwargs) -> None: ...
    @property
    def max(self) -> None:
        """
        :type: None
        """
    @property
    def min(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 56
    dimension = 2
    unitSquare: pxr.Gf.Range2d # value = Gf.Range2d(Gf.Vec2d(0.0, 0.0), Gf.Vec2d(1.0, 1.0))
    pass
class Range2f(Boost.Python.instance):
    @staticmethod
    def Contains(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCorner(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDistanceSquared(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIntersection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMidpoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def GetQuadrant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUnion(*args, **kwargs) -> None: ...
    @staticmethod
    def IntersectWith(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def UnionWith(*args, **kwargs) -> None: ...
    @property
    def max(self) -> None:
        """
        :type: None
        """
    @property
    def min(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    dimension = 2
    unitSquare: pxr.Gf.Range2f # value = Gf.Range2f(Gf.Vec2f(0.0, 0.0), Gf.Vec2f(1.0, 1.0))
    pass
class Range3d(Boost.Python.instance):
    @staticmethod
    def Contains(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCorner(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDistanceSquared(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIntersection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMidpoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOctant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUnion(*args, **kwargs) -> None: ...
    @staticmethod
    def IntersectWith(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def UnionWith(*args, **kwargs) -> None: ...
    @property
    def max(self) -> None:
        """
        :type: None
        """
    @property
    def min(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 72
    dimension = 3
    unitCube: pxr.Gf.Range3d # value = Gf.Range3d(Gf.Vec3d(0.0, 0.0, 0.0), Gf.Vec3d(1.0, 1.0, 1.0))
    pass
class Range3f(Boost.Python.instance):
    @staticmethod
    def Contains(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCorner(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDistanceSquared(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIntersection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMidpoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOctant(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUnion(*args, **kwargs) -> None: ...
    @staticmethod
    def IntersectWith(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def UnionWith(*args, **kwargs) -> None: ...
    @property
    def max(self) -> None:
        """
        :type: None
        """
    @property
    def min(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 48
    dimension = 3
    unitCube: pxr.Gf.Range3f # value = Gf.Range3f(Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(1.0, 1.0, 1.0))
    pass
class Ray(Boost.Python.instance):
    @staticmethod
    def FindClosestPoint(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPoint(*args, **kwargs) -> None: ...
    @staticmethod
    def Intersect(*args, **kwargs) -> typing.Any: 
        """
        Intersects the ray with the triangle formed by points p0,
        p1, and p2.  The first item in the tuple is true if the ray
        intersects the triangle. dist is the the parametric
        distance to the intersection point, the barycentric
        coordinates of the intersection point, and the front-facing
        flag. The barycentric coordinates are defined with respect
        to the three vertices taken in order.  The front-facing
        flag is True if the intersection hit the side of the
        triangle that is formed when the vertices are ordered
        counter-clockwise (right-hand rule).

        Barycentric coordinates are defined to sum to 1 and satisfy
        this relationsip:

            intersectionPoint = (barycentricCoords[0] * p0 +
                                 barycentricCoords[1] * p1 +
                                 barycentricCoords[2] * p2);
        ----------------------------------------------------------------------

        Intersects the ray with the Gf.Plane.  The first item in
        the returned tuple is true if the ray intersects the plane.
        dist is the parametric distance to the intersection point
        and frontfacing is true if the intersection is on the side
        of the plane toward which the plane's normal points.
        ----------------------------------------------------------------------

        Intersects the plane with an sphere. intersects is true if
        the ray intersects it at all within the sphere. If there is
        an intersection then enterDist and exitDist will be the
        parametric distances to the two intersection points.
        ----------------------------------------------------------------------

        Intersects the plane with an infinite cylinder. intersects
        is true if the ray intersects it at all within the
        sphere. If there is an intersection then enterDist and
        exitDist will be the parametric distances to the two
        intersection points.
        ----------------------------------------------------------------------

        Intersects the plane with an cylinder. intersects
        is true if the ray intersects it at all within the
        sphere. If there is an intersection then enterDist and
        exitDist will be the parametric distances to the two
        intersection points.
        ----------------------------------------------------------------------
        """
    @staticmethod
    def SetEnds(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPointAndDirection(*args, **kwargs) -> None: ...
    @staticmethod
    def Transform(*args, **kwargs) -> None: ...
    @property
    def direction(self) -> None:
        """
        :type: None
        """
    @property
    def startPoint(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 72
    pass
class Rect2i(Boost.Python.instance):
    @staticmethod
    def Contains(*args, **kwargs) -> None: ...
    @staticmethod
    def GetArea(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCenter(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHeight(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIntersection(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMaxX(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMaxY(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMinX(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMinY(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUnion(*args, **kwargs) -> None: ...
    @staticmethod
    def GetWidth(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def IsNull(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValid(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMax(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMaxX(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMaxY(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMin(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMinX(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMinY(*args, **kwargs) -> None: ...
    @staticmethod
    def Translate(*args, **kwargs) -> None: ...
    @property
    def max(self) -> None:
        """
        :type: None
        """
    @property
    def maxX(self) -> None:
        """
        :type: None
        """
    @property
    def maxY(self) -> None:
        """
        :type: None
        """
    @property
    def min(self) -> None:
        """
        :type: None
        """
    @property
    def minX(self) -> None:
        """
        :type: None
        """
    @property
    def minY(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class Rotation(Boost.Python.instance):
    """
    3-space rotation
    """
    @staticmethod
    def Decompose(*args, **kwargs) -> None: ...
    @staticmethod
    def DecomposeRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def DecomposeRotation3(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAngle(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def GetQuat(*args, **kwargs) -> None: ...
    @staticmethod
    def GetQuaternion(*args, **kwargs) -> None: ...
    @staticmethod
    def MatchClosestEulerRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def RotateOntoProjected(*args, **kwargs) -> None: ...
    @staticmethod
    def SetAxisAngle(*args, **kwargs) -> None: ...
    @staticmethod
    def SetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def SetQuat(*args, **kwargs) -> None: ...
    @staticmethod
    def SetQuaternion(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotateInto(*args, **kwargs) -> None: ...
    @staticmethod
    def TransformDir(*args, **kwargs) -> None: ...
    @property
    def angle(self) -> None:
        """
        :type: None
        """
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 56
    pass
class Size2(Boost.Python.instance):
    """
    A 2D size class
    """
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    __instance_size__ = 40
    dimension = 2
    pass
class Size3(Boost.Python.instance):
    """
    A 3D size class
    """
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    dimension = 3
    pass
class Transform(Boost.Python.instance):
    @staticmethod
    def GetMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPivotOrientation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPivotPosition(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetScale(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTranslation(*args, **kwargs) -> None: ...
    @staticmethod
    def Set(*args, **kwargs) -> None: 
        """
        Set method used by old 2x code. (Deprecated)
        """
    @staticmethod
    def SetIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMatrix(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPivotOrientation(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPivotPosition(*args, **kwargs) -> None: ...
    @staticmethod
    def SetRotation(*args, **kwargs) -> None: ...
    @staticmethod
    def SetScale(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTranslation(*args, **kwargs) -> None: ...
    @property
    def pivotOrientation(self) -> None:
        """
        :type: None
        """
    @property
    def pivotPosition(self) -> None:
        """
        :type: None
        """
    @property
    def rotation(self) -> None:
        """
        :type: None
        """
    @property
    def scale(self) -> None:
        """
        :type: None
        """
    @property
    def translation(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 160
    pass
class Vec2d(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 2
    pass
class Vec2f(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 2
    pass
class Vec2h(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 2
    pass
class Vec2i(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 2
    pass
class Vec3d(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def BuildOrthonormalFrame(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCross(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def OrthogonalizeBasis(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def ZAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 3
    pass
class Vec3f(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def BuildOrthonormalFrame(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCross(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def OrthogonalizeBasis(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def ZAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 3
    pass
class Vec3h(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def BuildOrthonormalFrame(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCross(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def OrthogonalizeBasis(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def ZAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 3
    pass
class Vec3i(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def ZAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 3
    pass
class Vec4d(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def WAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def ZAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 4
    pass
class Vec4f(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def WAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def ZAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 4
    pass
class Vec4h(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLength(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNormalized(*args, **kwargs) -> None: ...
    @staticmethod
    def GetProjection(*args, **kwargs) -> None: ...
    @staticmethod
    def Normalize(*args, **kwargs) -> None: ...
    @staticmethod
    def WAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def ZAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 4
    pass
class Vec4i(Boost.Python.instance):
    @staticmethod
    def Axis(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDot(*args, **kwargs) -> None: ...
    @staticmethod
    def WAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def XAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def YAxis(*args, **kwargs) -> None: ...
    @staticmethod
    def ZAxis(*args, **kwargs) -> None: ...
    __isGfVec = True
    __safe_for_unpickling__ = True
    dimension = 4
    pass
def Abs(*args, **kwargs) -> None:
    pass
def Absf(f) -> float:
    """
    f : float

    Use instead of Abs() to return the absolute value of f as a float instead of a double.
    """
def ApplyGamma(*args, **kwargs) -> None:
    pass
def Ceil(*args, **kwargs) -> None:
    pass
def Ceilf(f) -> float:
    """
    f : float

    Use instead of Ceil() to return the ceiling of f as a float instead of a double.
    """
def Clamp(*args, **kwargs) -> None:
    pass
def Clampf(f) -> float:
    """
    f : float

    Use instead of Clamp() to return the clamped value of f as a float instead of a double.
    """
def CompDiv(*args, **kwargs) -> None:
    pass
def CompMult(*args, **kwargs) -> None:
    pass
def ConvertDisplayToLinear(*args, **kwargs) -> None:
    pass
def ConvertLinearToDisplay(*args, **kwargs) -> None:
    pass
def Cross(*args, **kwargs) -> None:
    pass
def DegreesToRadians(*args, **kwargs) -> None:
    pass
def Dot(*args, **kwargs) -> None:
    pass
def Exp(*args, **kwargs) -> None:
    pass
def Expf(f) -> float:
    """
    f : float

    Use instead of Exp() to return the exponent of f as a float instead of a double.
    """
def FindClosestPoints(*args, **kwargs) -> typing.Any:
    """
    l1 : GfLine
    l2 : GfLine

    Computes the closest points between two lines, returning a tuple.  The first item in the tuple is true if the linesintersect.  The two points are returned in p1 and p2.  The parametric distance of each point on the lines is returned in t1 and t2.
    ----------------------------------------------------------------------

    l1 : GfLine
    s2 : GfLineSeg

    Computes the closest points between a line and a line segment, returning a tuple. The first item in the tuple is true if they intersect. The two points are returned in p1 and p2.  The parametric distance of each point on the line and line segment is returned in t1 and t2.
    ----------------------------------------------------------------------

    l1 : GfLineSeg
    l2 : GfLineSeg

    Computes the closest points between two line segments, returning a tuple.  The first item in the tuple is true if they intersect.  The two points are returned in p1 and p2.  The parametric distance of each point on the line and line segment is returned in t1 and t2.
    ----------------------------------------------------------------------

    r1 : GfRay
    l2 : GfLine

    Computes the closest points between a ray and a line,
    returning a tuple. The first item in the tuple is true if they intersect. The two points are returned in p1 and p2.
    The parametric distance of each point on the ray and line is
    returned in t1 and t2.
    ----------------------------------------------------------------------

    r1 : GfRay
    s2 : GfLineSeg

    Computes the closest points between a ray and a line segment,
    returning a tuple. The first item in the tuple is true if they intersect. The two points are returned in p1 and p2.
    The parametric distance of each point on the ray and line
    segment is returned in t1 and t2.
    ----------------------------------------------------------------------
    """
def FitPlaneToPoints(*args, **kwargs) -> None:
    pass
def Floor(*args, **kwargs) -> None:
    pass
def Floorf(f) -> float:
    """
    f : float

    Use instead of Floor() to return the floor of f as a float instead of a double.
    """
def GetComplement(*args, **kwargs) -> None:
    pass
def GetDisplayGamma(*args, **kwargs) -> None:
    pass
def GetHomogenized(*args, **kwargs) -> None:
    pass
def GetLength(*args, **kwargs) -> None:
    pass
def GetNormalized(*args, **kwargs) -> None:
    pass
def GetProjection(*args, **kwargs) -> None:
    pass
def HomogeneousCross(*args, **kwargs) -> None:
    pass
def IsClose(*args, **kwargs) -> None:
    pass
def Lerp(*args, **kwargs) -> None:
    pass
def Lerpf(f) -> float:
    """
    f : float

    Use instead of Lerp() to return the linear interpolation of f as a float instead of a double.
    """
def Log(*args, **kwargs) -> None:
    pass
def Logf(f) -> float:
    """
    f : float

    Use instead of Log() to return the logarithm of f as a float instead of a double.
    """
def Max(*args, **kwargs) -> None:
    pass
def Min(*args, **kwargs) -> None:
    pass
def Mod(*args, **kwargs) -> None:
    pass
def Modf(f) -> float:
    """
    f : float

    Use instead of Mod() to return the modulus of f as a float instead of a double.
    """
def Normalize(*args, **kwargs) -> None:
    pass
def Pow(*args, **kwargs) -> None:
    pass
def Powf(f) -> float:
    """
    f : float

    Use instead of Pow() to return the power of f as a float instead of a double.
    """
def Project(*args, **kwargs) -> None:
    pass
def RadiansToDegrees(*args, **kwargs) -> None:
    pass
def Round(*args, **kwargs) -> None:
    pass
def Roundf(f) -> float:
    """
    f : float

    Use instead of Round() to return the rounded value of f as a float instead of a double.
    """
def Sgn(*args, **kwargs) -> None:
    pass
def Slerp(*args, **kwargs) -> None:
    pass
def SmoothStep(*args, **kwargs) -> None:
    pass
def Sqr(*args, **kwargs) -> None:
    pass
def Sqrt(*args, **kwargs) -> None:
    pass
def Sqrtf(f) -> float:
    """
    f : float

    Use instead of Sqrt() to return the square root of f as a float instead of a double.
    """
def _HalfRoundTrip(*args, **kwargs) -> None:
    pass
MIN_ORTHO_TOLERANCE = 1e-06
MIN_VECTOR_LENGTH = 1e-10
__MFB_FULL_PACKAGE_NAME = 'gf'
