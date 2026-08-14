from __future__ import annotations
import pxr.UsdPhysics._usdPhysics
import typing
import Boost.Python
import pxr.Usd
import pxr.UsdGeom
import pxr.UsdPhysics

__all__ = [
    "ArticulationDesc",
    "ArticulationDescVector",
    "ArticulationRootAPI",
    "Axis",
    "Capsule1ShapeDesc",
    "Capsule1ShapeDescVector",
    "CapsuleShapeDesc",
    "CapsuleShapeDescVector",
    "CollisionAPI",
    "CollisionGroup",
    "CollisionGroupDesc",
    "CollisionGroupDescVector",
    "CollisionGroupTable",
    "ConeShapeDesc",
    "ConeShapeDescVector",
    "CubeShapeDesc",
    "CubeShapeDescVector",
    "CustomJointDesc",
    "CustomJointDescVector",
    "CustomShapeDesc",
    "CustomShapeDescVector",
    "CustomUsdPhysicsTokens",
    "Cylinder1ShapeDesc",
    "Cylinder1ShapeDescVector",
    "CylinderShapeDesc",
    "CylinderShapeDescVector",
    "D6JointDesc",
    "D6JointDescVector",
    "DistanceJoint",
    "DistanceJointDesc",
    "DistanceJointDescVector",
    "DriveAPI",
    "FilteredPairsAPI",
    "FixedJoint",
    "FixedJointDesc",
    "FixedJointDescVector",
    "GetStageKilogramsPerUnit",
    "Joint",
    "JointDOF",
    "JointDesc",
    "JointDescVector",
    "JointDrive",
    "JointDriveDOFPair",
    "JointLimit",
    "JointLimitDOFPair",
    "LimitAPI",
    "LoadUsdPhysicsFromRange",
    "MassAPI",
    "MassUnits",
    "MassUnitsAre",
    "MaterialAPI",
    "MeshCollisionAPI",
    "MeshShapeDesc",
    "MeshShapeDescVector",
    "ObjectDesc",
    "ObjectType",
    "PhysicsCollectionMembershipQueryVector",
    "PhysicsJointDriveDOFVector",
    "PhysicsJointLimitDOFVector",
    "PhysicsSpherePointVector",
    "PlaneShapeDesc",
    "PlaneShapeDescVector",
    "PrismaticJoint",
    "PrismaticJointDesc",
    "PrismaticJointDescVector",
    "RevoluteJoint",
    "RevoluteJointDesc",
    "RevoluteJointDescVector",
    "RigidBodyAPI",
    "RigidBodyDesc",
    "RigidBodyDescVector",
    "RigidBodyMaterialDesc",
    "RigidBodyMaterialDescVector",
    "Scene",
    "SceneDesc",
    "SceneDescVector",
    "SetStageKilogramsPerUnit",
    "ShapeDesc",
    "SpherePoint",
    "SpherePointsShapeDesc",
    "SpherePointsShapeDescVector",
    "SphereShapeDesc",
    "SphereShapeDescVector",
    "SphericalJoint",
    "SphericalJointDesc",
    "SphericalJointDescVector",
    "StageHasAuthoredKilogramsPerUnit",
    "Tokens"
]


class ArticulationDesc(ObjectDesc, Boost.Python.instance):
    @property
    def articulatedBodies(self) -> None:
        """
        :type: None
        """
    @property
    def articulatedJoints(self) -> None:
        """
        :type: None
        """
    @property
    def filteredCollisions(self) -> None:
        """
        :type: None
        """
    @property
    def rootPrims(self) -> None:
        """
        :type: None
        """
    pass
class ArticulationDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class ArticulationRootAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class Axis(Boost.Python.enum, int):
    X = pxr.UsdPhysics.Axis.X
    Y = pxr.UsdPhysics.Axis.Y
    Z = pxr.UsdPhysics.Axis.Z
    __slots__ = ()
    names = {'X': pxr.UsdPhysics.Axis.X, 'Y': pxr.UsdPhysics.Axis.Y, 'Z': pxr.UsdPhysics.Axis.Z}
    values = {0: pxr.UsdPhysics.Axis.X, 1: pxr.UsdPhysics.Axis.Y, 2: pxr.UsdPhysics.Axis.Z}
    pass
class Capsule1ShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    @property
    def bottomRadius(self) -> None:
        """
        :type: None
        """
    @property
    def halfHeight(self) -> None:
        """
        :type: None
        """
    @property
    def topRadius(self) -> None:
        """
        :type: None
        """
    pass
class Capsule1ShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CapsuleShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    @property
    def halfHeight(self) -> None:
        """
        :type: None
        """
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass
class CapsuleShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CollisionAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateCollisionEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateSimulationOwnerRel(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCollisionEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSimulationOwnerRel(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class CollisionGroup(pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def ComputeCollisionGroupTable(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateFilteredGroupsRel(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateInvertFilteredGroupsAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateMergeGroupNameAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCollidersCollectionAPI(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFilteredGroupsRel(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInvertFilteredGroupsAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMergeGroupNameAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CollisionGroupDesc(ObjectDesc, Boost.Python.instance):
    @property
    def filteredGroups(self) -> None:
        """
        :type: None
        """
    @property
    def invertFilteredGroups(self) -> None:
        """
        :type: None
        """
    @property
    def mergeGroupName(self) -> None:
        """
        :type: None
        """
    @property
    def mergedGroups(self) -> None:
        """
        :type: None
        """
    pass
class CollisionGroupDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CollisionGroupTable(Boost.Python.instance):
    @staticmethod
    def GetGroups(*args, **kwargs) -> None: ...
    @staticmethod
    def IsCollisionEnabled(*args, **kwargs) -> None: ...
    __instance_size__ = 88
    pass
class ConeShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    @property
    def halfHeight(self) -> None:
        """
        :type: None
        """
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass
class ConeShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CubeShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def halfExtents(self) -> None:
        """
        :type: None
        """
    pass
class CubeShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CustomJointDesc(JointDesc, ObjectDesc, Boost.Python.instance):
    pass
class CustomJointDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CustomShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def customGeometryToken(self) -> None:
        """
        :type: None
        """
    pass
class CustomShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CustomUsdPhysicsTokens(Boost.Python.instance):
    @property
    def instancerTokens(self) -> None:
        """
        :type: None
        """
    @property
    def jointTokens(self) -> None:
        """
        :type: None
        """
    @property
    def shapeTokens(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 48
    pass
class Cylinder1ShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    @property
    def bottomRadius(self) -> None:
        """
        :type: None
        """
    @property
    def halfHeight(self) -> None:
        """
        :type: None
        """
    @property
    def topRadius(self) -> None:
        """
        :type: None
        """
    pass
class Cylinder1ShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class CylinderShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    @property
    def halfHeight(self) -> None:
        """
        :type: None
        """
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass
class CylinderShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class D6JointDesc(JointDesc, ObjectDesc, Boost.Python.instance):
    @property
    def jointDrives(self) -> None:
        """
        :type: None
        """
    @property
    def jointLimits(self) -> None:
        """
        :type: None
        """
    pass
class D6JointDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class DistanceJoint(Joint, pxr.UsdGeom.Imageable, pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def CreateMaxDistanceAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateMinDistanceAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMaxDistanceAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMinDistanceAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class DistanceJointDesc(JointDesc, ObjectDesc, Boost.Python.instance):
    @property
    def limit(self) -> None:
        """
        :type: None
        """
    @property
    def maxEnabled(self) -> None:
        """
        :type: None
        """
    @property
    def minEnabled(self) -> None:
        """
        :type: None
        """
    pass
class DistanceJointDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class DriveAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDampingAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateMaxForceAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateStiffnessAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateTargetPositionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateTargetVelocityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateTypeAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAll(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDampingAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMaxForceAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetStiffnessAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTargetPositionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTargetVelocityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTypeAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPhysicsDriveAPIPath(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class FilteredPairsAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateFilteredPairsRel(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFilteredPairsRel(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class FixedJoint(Joint, pxr.UsdGeom.Imageable, pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class FixedJointDesc(JointDesc, ObjectDesc, Boost.Python.instance):
    pass
class FixedJointDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class Joint(pxr.UsdGeom.Imageable, pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def CreateBody0Rel(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateBody1Rel(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateBreakForceAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateBreakTorqueAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateCollisionEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExcludeFromArticulationAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateJointEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLocalPos0Attr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLocalPos1Attr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLocalRot0Attr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLocalRot1Attr(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBody0Rel(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBody1Rel(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBreakForceAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBreakTorqueAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCollisionEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetExcludeFromArticulationAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetJointEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLocalPos0Attr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLocalPos1Attr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLocalRot0Attr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLocalRot1Attr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class JointDOF(Boost.Python.enum, int):
    Distance = pxr.UsdPhysics.JointDOF.Distance
    RotX = pxr.UsdPhysics.JointDOF.RotX
    RotY = pxr.UsdPhysics.JointDOF.RotY
    RotZ = pxr.UsdPhysics.JointDOF.RotZ
    TransX = pxr.UsdPhysics.JointDOF.TransX
    TransY = pxr.UsdPhysics.JointDOF.TransY
    TransZ = pxr.UsdPhysics.JointDOF.TransZ
    __slots__ = ()
    names = {'Distance': pxr.UsdPhysics.JointDOF.Distance, 'TransX': pxr.UsdPhysics.JointDOF.TransX, 'TransY': pxr.UsdPhysics.JointDOF.TransY, 'TransZ': pxr.UsdPhysics.JointDOF.TransZ, 'RotX': pxr.UsdPhysics.JointDOF.RotX, 'RotY': pxr.UsdPhysics.JointDOF.RotY, 'RotZ': pxr.UsdPhysics.JointDOF.RotZ}
    values = {0: pxr.UsdPhysics.JointDOF.Distance, 1: pxr.UsdPhysics.JointDOF.TransX, 2: pxr.UsdPhysics.JointDOF.TransY, 3: pxr.UsdPhysics.JointDOF.TransZ, 4: pxr.UsdPhysics.JointDOF.RotX, 5: pxr.UsdPhysics.JointDOF.RotY, 6: pxr.UsdPhysics.JointDOF.RotZ}
    pass
class JointDesc(ObjectDesc, Boost.Python.instance):
    @property
    def body0(self) -> None:
        """
        :type: None
        """
    @property
    def body1(self) -> None:
        """
        :type: None
        """
    @property
    def breakForce(self) -> None:
        """
        :type: None
        """
    @property
    def breakTorque(self) -> None:
        """
        :type: None
        """
    @property
    def collisionEnabled(self) -> None:
        """
        :type: None
        """
    @property
    def excludeFromArticulation(self) -> None:
        """
        :type: None
        """
    @property
    def jointEnabled(self) -> None:
        """
        :type: None
        """
    @property
    def localPose0Orientation(self) -> None:
        """
        :type: None
        """
    @property
    def localPose0Position(self) -> None:
        """
        :type: None
        """
    @property
    def localPose1Orientation(self) -> None:
        """
        :type: None
        """
    @property
    def localPose1Position(self) -> None:
        """
        :type: None
        """
    @property
    def rel0(self) -> None:
        """
        :type: None
        """
    @property
    def rel1(self) -> None:
        """
        :type: None
        """
    pass
class JointDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class JointDrive(Boost.Python.instance):
    @property
    def acceleration(self) -> None:
        """
        :type: None
        """
    @property
    def damping(self) -> None:
        """
        :type: None
        """
    @property
    def enabled(self) -> None:
        """
        :type: None
        """
    @property
    def forceLimit(self) -> None:
        """
        :type: None
        """
    @property
    def stiffness(self) -> None:
        """
        :type: None
        """
    @property
    def targetPosition(self) -> None:
        """
        :type: None
        """
    @property
    def targetVelocity(self) -> None:
        """
        :type: None
        """
    pass
class JointDriveDOFPair(Boost.Python.instance):
    @property
    def first(self) -> None:
        """
        :type: None
        """
    @property
    def second(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 56
    pass
class JointLimit(Boost.Python.instance):
    @property
    def enabled(self) -> None:
        """
        :type: None
        """
    @property
    def lower(self) -> None:
        """
        :type: None
        """
    @property
    def upper(self) -> None:
        """
        :type: None
        """
    pass
class JointLimitDOFPair(Boost.Python.instance):
    @property
    def first(self) -> None:
        """
        :type: None
        """
    @property
    def second(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class LimitAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateHighAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLowAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAll(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHighAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLowAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPhysicsLimitAPIPath(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class MassAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateCenterOfMassAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDensityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDiagonalInertiaAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateMassAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreatePrincipalAxesAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCenterOfMassAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDensityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDiagonalInertiaAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMassAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrincipalAxesAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class MassUnits(Boost.Python.instance):
    grams = 0.001
    kilograms = 1.0
    slugs = 14.5939
    pass
class MaterialAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDensityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDynamicFrictionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateRestitutionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateStaticFrictionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDensityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDynamicFrictionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRestitutionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetStaticFrictionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class MeshCollisionAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateApproximationAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetApproximationAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class MeshShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def approximation(self) -> None:
        """
        :type: None
        """
    @property
    def doubleSided(self) -> None:
        """
        :type: None
        """
    @property
    def meshScale(self) -> None:
        """
        :type: None
        """
    pass
class MeshShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class ObjectDesc(Boost.Python.instance):
    @property
    def isValid(self) -> None:
        """
        :type: None
        """
    @property
    def primPath(self) -> None:
        """
        :type: None
        """
    @property
    def type(self) -> None:
        """
        :type: None
        """
    pass
class ObjectType(Boost.Python.enum, int):
    Articulation = pxr.UsdPhysics.ObjectType.Articulation
    Capsule1Shape = pxr.UsdPhysics.ObjectType.Capsule1Shape
    CapsuleShape = pxr.UsdPhysics.ObjectType.CapsuleShape
    CollisionGroup = pxr.UsdPhysics.ObjectType.CollisionGroup
    ConeShape = pxr.UsdPhysics.ObjectType.ConeShape
    CubeShape = pxr.UsdPhysics.ObjectType.CubeShape
    CustomJoint = pxr.UsdPhysics.ObjectType.CustomJoint
    CustomShape = pxr.UsdPhysics.ObjectType.CustomShape
    Cylinder1Shape = pxr.UsdPhysics.ObjectType.Cylinder1Shape
    CylinderShape = pxr.UsdPhysics.ObjectType.CylinderShape
    D6Joint = pxr.UsdPhysics.ObjectType.D6Joint
    DistanceJoint = pxr.UsdPhysics.ObjectType.DistanceJoint
    FixedJoint = pxr.UsdPhysics.ObjectType.FixedJoint
    MeshShape = pxr.UsdPhysics.ObjectType.MeshShape
    PlaneShape = pxr.UsdPhysics.ObjectType.PlaneShape
    PrismaticJoint = pxr.UsdPhysics.ObjectType.PrismaticJoint
    RevoluteJoint = pxr.UsdPhysics.ObjectType.RevoluteJoint
    RigidBody = pxr.UsdPhysics.ObjectType.RigidBody
    RigidBodyMaterial = pxr.UsdPhysics.ObjectType.RigidBodyMaterial
    Scene = pxr.UsdPhysics.ObjectType.Scene
    SpherePointsShape = pxr.UsdPhysics.ObjectType.SpherePointsShape
    SphereShape = pxr.UsdPhysics.ObjectType.SphereShape
    SphericalJoint = pxr.UsdPhysics.ObjectType.SphericalJoint
    Undefined = pxr.UsdPhysics.ObjectType.Undefined
    __slots__ = ()
    names = {'Undefined': pxr.UsdPhysics.ObjectType.Undefined, 'Scene': pxr.UsdPhysics.ObjectType.Scene, 'RigidBody': pxr.UsdPhysics.ObjectType.RigidBody, 'SphereShape': pxr.UsdPhysics.ObjectType.SphereShape, 'CubeShape': pxr.UsdPhysics.ObjectType.CubeShape, 'CapsuleShape': pxr.UsdPhysics.ObjectType.CapsuleShape, 'Capsule1Shape': pxr.UsdPhysics.ObjectType.Capsule1Shape, 'CylinderShape': pxr.UsdPhysics.ObjectType.CylinderShape, 'Cylinder1Shape': pxr.UsdPhysics.ObjectType.Cylinder1Shape, 'ConeShape': pxr.UsdPhysics.ObjectType.ConeShape, 'MeshShape': pxr.UsdPhysics.ObjectType.MeshShape, 'PlaneShape': pxr.UsdPhysics.ObjectType.PlaneShape, 'CustomShape': pxr.UsdPhysics.ObjectType.CustomShape, 'SpherePointsShape': pxr.UsdPhysics.ObjectType.SpherePointsShape, 'FixedJoint': pxr.UsdPhysics.ObjectType.FixedJoint, 'RevoluteJoint': pxr.UsdPhysics.ObjectType.RevoluteJoint, 'PrismaticJoint': pxr.UsdPhysics.ObjectType.PrismaticJoint, 'SphericalJoint': pxr.UsdPhysics.ObjectType.SphericalJoint, 'DistanceJoint': pxr.UsdPhysics.ObjectType.DistanceJoint, 'D6Joint': pxr.UsdPhysics.ObjectType.D6Joint, 'CustomJoint': pxr.UsdPhysics.ObjectType.CustomJoint, 'RigidBodyMaterial': pxr.UsdPhysics.ObjectType.RigidBodyMaterial, 'Articulation': pxr.UsdPhysics.ObjectType.Articulation, 'CollisionGroup': pxr.UsdPhysics.ObjectType.CollisionGroup}
    values = {0: pxr.UsdPhysics.ObjectType.Undefined, 1: pxr.UsdPhysics.ObjectType.Scene, 2: pxr.UsdPhysics.ObjectType.RigidBody, 3: pxr.UsdPhysics.ObjectType.SphereShape, 4: pxr.UsdPhysics.ObjectType.CubeShape, 5: pxr.UsdPhysics.ObjectType.CapsuleShape, 6: pxr.UsdPhysics.ObjectType.Capsule1Shape, 7: pxr.UsdPhysics.ObjectType.CylinderShape, 8: pxr.UsdPhysics.ObjectType.Cylinder1Shape, 9: pxr.UsdPhysics.ObjectType.ConeShape, 10: pxr.UsdPhysics.ObjectType.MeshShape, 11: pxr.UsdPhysics.ObjectType.PlaneShape, 12: pxr.UsdPhysics.ObjectType.CustomShape, 13: pxr.UsdPhysics.ObjectType.SpherePointsShape, 14: pxr.UsdPhysics.ObjectType.FixedJoint, 15: pxr.UsdPhysics.ObjectType.RevoluteJoint, 16: pxr.UsdPhysics.ObjectType.PrismaticJoint, 17: pxr.UsdPhysics.ObjectType.SphericalJoint, 18: pxr.UsdPhysics.ObjectType.DistanceJoint, 19: pxr.UsdPhysics.ObjectType.D6Joint, 20: pxr.UsdPhysics.ObjectType.CustomJoint, 21: pxr.UsdPhysics.ObjectType.RigidBodyMaterial, 22: pxr.UsdPhysics.ObjectType.Articulation, 23: pxr.UsdPhysics.ObjectType.CollisionGroup}
    pass
class PhysicsCollectionMembershipQueryVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class PhysicsJointDriveDOFVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class PhysicsJointLimitDOFVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class PhysicsSpherePointVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class PlaneShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    pass
class PlaneShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class PrismaticJoint(Joint, pxr.UsdGeom.Imageable, pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def CreateAxisAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLowerLimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateUpperLimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAxisAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLowerLimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUpperLimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class PrismaticJointDesc(JointDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    @property
    def drive(self) -> None:
        """
        :type: None
        """
    @property
    def limit(self) -> None:
        """
        :type: None
        """
    pass
class PrismaticJointDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class RevoluteJoint(Joint, pxr.UsdGeom.Imageable, pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def CreateAxisAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLowerLimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateUpperLimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAxisAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLowerLimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUpperLimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class RevoluteJointDesc(JointDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    @property
    def drive(self) -> None:
        """
        :type: None
        """
    @property
    def limit(self) -> None:
        """
        :type: None
        """
    pass
class RevoluteJointDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class RigidBodyAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    class MassInformation(Boost.Python.instance):
        @property
        def centerOfMass(self) -> None:
            """
            :type: None
            """
        @property
        def inertia(self) -> None:
            """
            :type: None
            """
        @property
        def localPos(self) -> None:
            """
            :type: None
            """
        @property
        def localRot(self) -> None:
            """
            :type: None
            """
        @property
        def volume(self) -> None:
            """
            :type: None
            """
        __instance_size__ = 104
        pass
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeMassProperties(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateAngularVelocityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateKinematicEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateRigidBodyEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateSimulationOwnerRel(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateStartsAsleepAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateVelocityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAngularVelocityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetKinematicEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRigidBodyEnabledAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSimulationOwnerRel(*args, **kwargs) -> None: ...
    @staticmethod
    def GetStartsAsleepAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVelocityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class RigidBodyDesc(ObjectDesc, Boost.Python.instance):
    @property
    def angularVelocity(self) -> None:
        """
        :type: None
        """
    @property
    def collisions(self) -> None:
        """
        :type: None
        """
    @property
    def filteredCollisions(self) -> None:
        """
        :type: None
        """
    @property
    def kinematicBody(self) -> None:
        """
        :type: None
        """
    @property
    def linearVelocity(self) -> None:
        """
        :type: None
        """
    @property
    def position(self) -> None:
        """
        :type: None
        """
    @property
    def rigidBodyEnabled(self) -> None:
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
    def simulationOwners(self) -> None:
        """
        :type: None
        """
    @property
    def startsAsleep(self) -> None:
        """
        :type: None
        """
    pass
class RigidBodyDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class RigidBodyMaterialDesc(ObjectDesc, Boost.Python.instance):
    @property
    def density(self) -> None:
        """
        :type: None
        """
    @property
    def dynamicFriction(self) -> None:
        """
        :type: None
        """
    @property
    def restitution(self) -> None:
        """
        :type: None
        """
    @property
    def staticFriction(self) -> None:
        """
        :type: None
        """
    pass
class RigidBodyMaterialDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class Scene(pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def CreateGravityDirectionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateGravityMagnitudeAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetGravityDirectionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetGravityMagnitudeAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class SceneDesc(ObjectDesc, Boost.Python.instance):
    @property
    def gravityDirection(self) -> None:
        """
        :type: None
        """
    @property
    def gravityMagnitude(self) -> None:
        """
        :type: None
        """
    pass
class SceneDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class ShapeDesc(ObjectDesc, Boost.Python.instance):
    @property
    def collisionEnabled(self) -> None:
        """
        :type: None
        """
    @property
    def collisionGroups(self) -> None:
        """
        :type: None
        """
    @property
    def filteredCollisions(self) -> None:
        """
        :type: None
        """
    @property
    def localPos(self) -> None:
        """
        :type: None
        """
    @property
    def localRot(self) -> None:
        """
        :type: None
        """
    @property
    def localScale(self) -> None:
        """
        :type: None
        """
    @property
    def materials(self) -> None:
        """
        :type: None
        """
    @property
    def rigidBody(self) -> None:
        """
        :type: None
        """
    @property
    def simulationOwners(self) -> None:
        """
        :type: None
        """
    pass
class SpherePoint(Boost.Python.instance):
    @property
    def center(self) -> None:
        """
        :type: None
        """
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass
class SpherePointsShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def spherePoints(self) -> None:
        """
        :type: None
        """
    pass
class SpherePointsShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class SphereShapeDesc(ShapeDesc, ObjectDesc, Boost.Python.instance):
    @property
    def radius(self) -> None:
        """
        :type: None
        """
    pass
class SphereShapeDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class SphericalJoint(Joint, pxr.UsdGeom.Imageable, pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def CreateAxisAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateConeAngle0LimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateConeAngle1LimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAxisAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetConeAngle0LimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetConeAngle1LimitAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class SphericalJointDesc(JointDesc, ObjectDesc, Boost.Python.instance):
    @property
    def axis(self) -> None:
        """
        :type: None
        """
    @property
    def limit(self) -> None:
        """
        :type: None
        """
    pass
class SphericalJointDescVector(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class Tokens(Boost.Python.instance):
    PhysicsArticulationRootAPI = 'PhysicsArticulationRootAPI'
    PhysicsCollisionAPI = 'PhysicsCollisionAPI'
    PhysicsCollisionGroup = 'PhysicsCollisionGroup'
    PhysicsDistanceJoint = 'PhysicsDistanceJoint'
    PhysicsDriveAPI = 'PhysicsDriveAPI'
    PhysicsFilteredPairsAPI = 'PhysicsFilteredPairsAPI'
    PhysicsFixedJoint = 'PhysicsFixedJoint'
    PhysicsJoint = 'PhysicsJoint'
    PhysicsLimitAPI = 'PhysicsLimitAPI'
    PhysicsMassAPI = 'PhysicsMassAPI'
    PhysicsMaterialAPI = 'PhysicsMaterialAPI'
    PhysicsMeshCollisionAPI = 'PhysicsMeshCollisionAPI'
    PhysicsPrismaticJoint = 'PhysicsPrismaticJoint'
    PhysicsRevoluteJoint = 'PhysicsRevoluteJoint'
    PhysicsRigidBodyAPI = 'PhysicsRigidBodyAPI'
    PhysicsScene = 'PhysicsScene'
    PhysicsSphericalJoint = 'PhysicsSphericalJoint'
    acceleration = 'acceleration'
    angular = 'angular'
    boundingCube = 'boundingCube'
    boundingSphere = 'boundingSphere'
    colliders = 'colliders'
    convexDecomposition = 'convexDecomposition'
    convexHull = 'convexHull'
    distance = 'distance'
    drive = 'drive'
    drive_MultipleApplyTemplate_PhysicsDamping = 'drive:__INSTANCE_NAME__:physics:damping'
    drive_MultipleApplyTemplate_PhysicsMaxForce = 'drive:__INSTANCE_NAME__:physics:maxForce'
    drive_MultipleApplyTemplate_PhysicsStiffness = 'drive:__INSTANCE_NAME__:physics:stiffness'
    drive_MultipleApplyTemplate_PhysicsTargetPosition = 'drive:__INSTANCE_NAME__:physics:targetPosition'
    drive_MultipleApplyTemplate_PhysicsTargetVelocity = 'drive:__INSTANCE_NAME__:physics:targetVelocity'
    drive_MultipleApplyTemplate_PhysicsType = 'drive:__INSTANCE_NAME__:physics:type'
    force = 'force'
    kilogramsPerUnit = 'kilogramsPerUnit'
    limit = 'limit'
    limit_MultipleApplyTemplate_PhysicsHigh = 'limit:__INSTANCE_NAME__:physics:high'
    limit_MultipleApplyTemplate_PhysicsLow = 'limit:__INSTANCE_NAME__:physics:low'
    linear = 'linear'
    meshSimplification = 'meshSimplification'
    none = 'none'
    physicsAngularVelocity = 'physics:angularVelocity'
    physicsApproximation = 'physics:approximation'
    physicsAxis = 'physics:axis'
    physicsBody0 = 'physics:body0'
    physicsBody1 = 'physics:body1'
    physicsBreakForce = 'physics:breakForce'
    physicsBreakTorque = 'physics:breakTorque'
    physicsCenterOfMass = 'physics:centerOfMass'
    physicsCollisionEnabled = 'physics:collisionEnabled'
    physicsConeAngle0Limit = 'physics:coneAngle0Limit'
    physicsConeAngle1Limit = 'physics:coneAngle1Limit'
    physicsDensity = 'physics:density'
    physicsDiagonalInertia = 'physics:diagonalInertia'
    physicsDynamicFriction = 'physics:dynamicFriction'
    physicsExcludeFromArticulation = 'physics:excludeFromArticulation'
    physicsFilteredGroups = 'physics:filteredGroups'
    physicsFilteredPairs = 'physics:filteredPairs'
    physicsGravityDirection = 'physics:gravityDirection'
    physicsGravityMagnitude = 'physics:gravityMagnitude'
    physicsInvertFilteredGroups = 'physics:invertFilteredGroups'
    physicsJointEnabled = 'physics:jointEnabled'
    physicsKinematicEnabled = 'physics:kinematicEnabled'
    physicsLocalPos0 = 'physics:localPos0'
    physicsLocalPos1 = 'physics:localPos1'
    physicsLocalRot0 = 'physics:localRot0'
    physicsLocalRot1 = 'physics:localRot1'
    physicsLowerLimit = 'physics:lowerLimit'
    physicsMass = 'physics:mass'
    physicsMaxDistance = 'physics:maxDistance'
    physicsMergeGroup = 'physics:mergeGroup'
    physicsMinDistance = 'physics:minDistance'
    physicsPrincipalAxes = 'physics:principalAxes'
    physicsRestitution = 'physics:restitution'
    physicsRigidBodyEnabled = 'physics:rigidBodyEnabled'
    physicsSimulationOwner = 'physics:simulationOwner'
    physicsStartsAsleep = 'physics:startsAsleep'
    physicsStaticFriction = 'physics:staticFriction'
    physicsUpperLimit = 'physics:upperLimit'
    physicsVelocity = 'physics:velocity'
    rotX = 'rotX'
    rotY = 'rotY'
    rotZ = 'rotZ'
    transX = 'transX'
    transY = 'transY'
    transZ = 'transZ'
    x = 'X'
    y = 'Y'
    z = 'Z'
    pass
class _CanApplyResult(Boost.Python.instance):
    @property
    def whyNot(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
def GetStageKilogramsPerUnit(*args, **kwargs) -> None:
    pass
def LoadUsdPhysicsFromRange(*args, **kwargs) -> None:
    pass
def MassUnitsAre(*args, **kwargs) -> None:
    pass
def SetStageKilogramsPerUnit(*args, **kwargs) -> None:
    pass
def StageHasAuthoredKilogramsPerUnit(*args, **kwargs) -> None:
    pass
__MFB_FULL_PACKAGE_NAME = 'usdPhysics'
