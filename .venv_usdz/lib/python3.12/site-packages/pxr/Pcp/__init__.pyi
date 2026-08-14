from __future__ import annotations
import pxr.Pcp._pcp
import typing
import Boost.Python
import pxr.Pcp
import pxr.Tf

__all__ = [
    "ArcType",
    "ArcTypeInherit",
    "ArcTypePayload",
    "ArcTypeReference",
    "ArcTypeRelocate",
    "ArcTypeRoot",
    "ArcTypeSpecialize",
    "ArcTypeVariant",
    "BuildPrimPropertyIndex",
    "Cache",
    "Dependency",
    "DependencyType",
    "DependencyTypeAncestral",
    "DependencyTypeAnyIncludingVirtual",
    "DependencyTypeAnyNonVirtual",
    "DependencyTypeDirect",
    "DependencyTypeNonVirtual",
    "DependencyTypeNone",
    "DependencyTypePartlyDirect",
    "DependencyTypePurelyDirect",
    "DependencyTypeRoot",
    "DependencyTypeVirtual",
    "DynamicFileFormatDependencyData",
    "ErrorArcCycle",
    "ErrorArcPermissionDenied",
    "ErrorArcToProhibitedChild",
    "ErrorBase",
    "ErrorCapacityExceeded",
    "ErrorInconsistentAttributeType",
    "ErrorInconsistentAttributeVariability",
    "ErrorInconsistentPropertyType",
    "ErrorInvalidAssetPath",
    "ErrorInvalidAssetPathBase",
    "ErrorInvalidAuthoredRelocation",
    "ErrorInvalidConflictingRelocation",
    "ErrorInvalidExternalTargetPath",
    "ErrorInvalidInstanceTargetPath",
    "ErrorInvalidPrimPath",
    "ErrorInvalidReferenceOffset",
    "ErrorInvalidSameTargetRelocations",
    "ErrorInvalidSublayerOffset",
    "ErrorInvalidSublayerOwnership",
    "ErrorInvalidSublayerPath",
    "ErrorInvalidTargetPath",
    "ErrorMutedAssetPath",
    "ErrorOpinionAtRelocationSource",
    "ErrorPrimPermissionDenied",
    "ErrorPropertyPermissionDenied",
    "ErrorRelocationBase",
    "ErrorSublayerCycle",
    "ErrorTargetPathBase",
    "ErrorTargetPermissionDenied",
    "ErrorType",
    "ErrorType_ArcCapacityExceeded",
    "ErrorType_ArcCycle",
    "ErrorType_ArcNamespaceDepthCapacityExceeded",
    "ErrorType_ArcPermissionDenied",
    "ErrorType_InconsistentAttributeType",
    "ErrorType_InconsistentAttributeVariability",
    "ErrorType_InconsistentPropertyType",
    "ErrorType_IndexCapacityExceeded",
    "ErrorType_InternalAssetPath",
    "ErrorType_InvalidAssetPath",
    "ErrorType_InvalidAuthoredRelocation",
    "ErrorType_InvalidConflictingRelocation",
    "ErrorType_InvalidExternalTargetPath",
    "ErrorType_InvalidInstanceTargetPath",
    "ErrorType_InvalidPrimPath",
    "ErrorType_InvalidReferenceOffset",
    "ErrorType_InvalidSameTargetRelocations",
    "ErrorType_InvalidSublayerOffset",
    "ErrorType_InvalidSublayerOwnership",
    "ErrorType_InvalidSublayerPath",
    "ErrorType_InvalidTargetPath",
    "ErrorType_InvalidVariantSelection",
    "ErrorType_MutedAssetPath",
    "ErrorType_OpinionAtRelocationSource",
    "ErrorType_PrimPermissionDenied",
    "ErrorType_PropertyPermissionDenied",
    "ErrorType_SublayerCycle",
    "ErrorType_TargetPermissionDenied",
    "ErrorType_UnresolvedPrimPath",
    "ErrorType_VariableExpressionError",
    "ErrorUnresolvedPrimPath",
    "ErrorVariableExpressionError",
    "ExpressionVariables",
    "ExpressionVariablesSource",
    "InstanceKey",
    "LayerRelocatesEditBuilder",
    "LayerStack",
    "LayerStackIdentifier",
    "LayerStackSite",
    "MapExpression",
    "MapFunction",
    "NodeRef",
    "PrimIndex",
    "PropertyIndex",
    "Site",
    "TranslatePathFromNodeToRoot",
    "TranslatePathFromRootToNode"
]


class ArcType(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Pcp.ArcTypeRoot, Pcp.ArcTypeInherit, Pcp.ArcTypeRelocate, Pcp.ArcTypeVariant, Pcp.ArcTypeReference, Pcp.ArcTypePayload, Pcp.ArcTypeSpecialize)
    pass
class Cache(Boost.Python.instance):
    @staticmethod
    def ComputeAttributeConnectionPaths(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeLayerStack(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputePrimIndex(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputePropertyIndex(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeRelationshipTargetPaths(*args, **kwargs) -> None: ...
    @staticmethod
    def FindAllLayerStacksUsingLayer(*args, **kwargs) -> None: ...
    @staticmethod
    def FindPrimIndex(*args, **kwargs) -> None: ...
    @staticmethod
    def FindPropertyIndex(*args, **kwargs) -> None: ...
    @staticmethod
    def FindSiteDependencies(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDynamicFileFormatArgumentDependencyData(*args, **kwargs) -> None: ...
    @staticmethod
    def GetExpressionVariablesFromLayerStackUsedByPrim(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLayerStackIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMutedLayers(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrimsUsingExpressionVariablesFromLayerStack(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUsedLayers(*args, **kwargs) -> None: ...
    @staticmethod
    def GetUsedLayersRevision(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVariantFallbacks(*args, **kwargs) -> None: ...
    @staticmethod
    def HasAnyDynamicFileFormatArgumentAttributeDependencies(*args, **kwargs) -> None: ...
    @staticmethod
    def HasAnyDynamicFileFormatArgumentFieldDependencies(*args, **kwargs) -> None: ...
    @staticmethod
    def HasRootLayerStack(*args, **kwargs) -> None: ...
    @staticmethod
    def IsInvalidAssetPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsInvalidSublayerIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def IsLayerMuted(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPayloadIncluded(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPossibleDynamicFileFormatArgumentAttribute(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPossibleDynamicFileFormatArgumentField(*args, **kwargs) -> None: ...
    @staticmethod
    def PrintStatistics(*args, **kwargs) -> None: ...
    @staticmethod
    def Reload(*args, **kwargs) -> None: ...
    @staticmethod
    def RequestLayerMuting(*args, **kwargs) -> None: ...
    @staticmethod
    def RequestPayloads(*args, **kwargs) -> None: ...
    @staticmethod
    def SetVariantFallbacks(*args, **kwargs) -> None: ...
    @staticmethod
    def UsesLayerStack(*args, **kwargs) -> None: ...
    @property
    def fileFormatTarget(self) -> None:
        """
        :type: None
        """
    @property
    def layerStack(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 488
    pass
class Dependency(Boost.Python.instance):
    @property
    def indexPath(self) -> None:
        """
        :type: None
        """
    @property
    def mapFunc(self) -> None:
        """
        :type: None
        """
    @property
    def sitePath(self) -> None:
        """
        :type: None
        """
    pass
class DependencyType(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Pcp.DependencyTypeNone, Pcp.DependencyTypeRoot, Pcp.DependencyTypePurelyDirect, Pcp.DependencyTypePartlyDirect, Pcp.DependencyTypeDirect, Pcp.DependencyTypeAncestral, Pcp.DependencyTypeVirtual, Pcp.DependencyTypeNonVirtual, Pcp.DependencyTypeAnyNonVirtual, Pcp.DependencyTypeAnyIncludingVirtual)
    pass
class DynamicFileFormatDependencyData(Boost.Python.instance):
    @staticmethod
    def CanAttributeDefaultValueChangeAffectFileFormatArguments(*args, **kwargs) -> None: ...
    @staticmethod
    def CanFieldChangeAffectFileFormatArguments(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRelevantAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRelevantFieldNames(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    pass
class ErrorArcCycle(ErrorBase, Boost.Python.instance):
    pass
class ErrorArcPermissionDenied(ErrorBase, Boost.Python.instance):
    pass
class ErrorArcToProhibitedChild(ErrorBase, Boost.Python.instance):
    pass
class ErrorCapacityExceeded(ErrorBase, Boost.Python.instance):
    pass
class ErrorInconsistentAttributeType(ErrorBase, Boost.Python.instance):
    pass
class ErrorInconsistentAttributeVariability(ErrorBase, Boost.Python.instance):
    pass
class ErrorInconsistentPropertyType(ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidAssetPath(ErrorInvalidAssetPathBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorMutedAssetPath(ErrorInvalidAssetPathBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidAssetPathBase(ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidAuthoredRelocation(ErrorRelocationBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidConflictingRelocation(ErrorRelocationBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidExternalTargetPath(ErrorTargetPathBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidInstanceTargetPath(ErrorTargetPathBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidPrimPath(ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidReferenceOffset(ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidSameTargetRelocations(ErrorRelocationBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidSublayerOffset(ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidSublayerOwnership(ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidSublayerPath(ErrorBase, Boost.Python.instance):
    pass
class ErrorInvalidTargetPath(ErrorTargetPathBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorOpinionAtRelocationSource(ErrorBase, Boost.Python.instance):
    pass
class ErrorPrimPermissionDenied(ErrorBase, Boost.Python.instance):
    pass
class ErrorPropertyPermissionDenied(ErrorBase, Boost.Python.instance):
    pass
class ErrorRelocationBase(ErrorBase, Boost.Python.instance):
    pass
class ErrorSublayerCycle(ErrorBase, Boost.Python.instance):
    pass
class ErrorTargetPermissionDenied(ErrorTargetPathBase, ErrorBase, Boost.Python.instance):
    pass
class ErrorTargetPathBase(ErrorBase, Boost.Python.instance):
    pass
class ErrorUnresolvedPrimPath(ErrorBase, Boost.Python.instance):
    pass
class ErrorVariableExpressionError(ErrorBase, Boost.Python.instance):
    pass
class ErrorBase(Boost.Python.instance):
    @property
    def errorType(self) -> None:
        """
        :type: None
        """
    @property
    def rootSite(self) -> None:
        """
        :type: None
        """
    pass
class ErrorType(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Pcp.ErrorType_ArcCycle, Pcp.ErrorType_ArcPermissionDenied, Pcp.ErrorType_IndexCapacityExceeded, Pcp.ErrorType_ArcCapacityExceeded, Pcp.ErrorType_ArcNamespaceDepthCapacityExceeded, Pcp.ErrorType_InconsistentPropertyType, Pcp.ErrorType_InconsistentAttributeType, Pcp.ErrorType_InconsistentAttributeVariability, Pcp.ErrorType_InternalAssetPath, Pcp.ErrorType_InvalidPrimPath, Pcp.ErrorType_InvalidAssetPath, Pcp.ErrorType_InvalidInstanceTargetPath, Pcp.ErrorType_InvalidExternalTargetPath, Pcp.ErrorType_InvalidTargetPath, Pcp.ErrorType_InvalidReferenceOffset, Pcp.ErrorType_InvalidSublayerOffset, Pcp.ErrorType_InvalidSublayerOwnership, Pcp.ErrorType_InvalidSublayerPath, Pcp.ErrorType_InvalidVariantSelection, Pcp.ErrorType_MutedAssetPath, Pcp.ErrorType_InvalidAuthoredRelocation, Pcp.ErrorType_InvalidConflictingRelocation, Pcp.ErrorType_InvalidSameTargetRelocations, Pcp.ErrorType_OpinionAtRelocationSource, Pcp.ErrorType_PrimPermissionDenied, Pcp.ErrorType_PropertyPermissionDenied, Pcp.ErrorType_SublayerCycle, Pcp.ErrorType_TargetPermissionDenied, Pcp.ErrorType_UnresolvedPrimPath, Pcp.ErrorType_VariableExpressionError)
    pass
class ExpressionVariables(Boost.Python.instance):
    @staticmethod
    def Compute(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSource(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVariables(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class ExpressionVariablesSource(Boost.Python.instance):
    @staticmethod
    def GetLayerStackIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def IsRootLayerStack(*args, **kwargs) -> None: ...
    @staticmethod
    def ResolveLayerStackIdentifier(*args, **kwargs) -> None: ...
    __instance_size__ = 40
    pass
class InstanceKey(Boost.Python.instance):
    __instance_size__ = 80
    pass
class LayerRelocatesEditBuilder(Boost.Python.instance):
    @staticmethod
    def GetEditedRelocatesMap(*args, **kwargs) -> None: ...
    @staticmethod
    def GetEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def Relocate(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveRelocate(*args, **kwargs) -> None: ...
    pass
class LayerStack(Boost.Python.instance):
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    @property
    def expressionVariableDependencies(self) -> None:
        """
        :type: None
        """
    @property
    def expressionVariables(self) -> None:
        """
        :type: None
        """
    @property
    def identifier(self) -> None:
        """
        :type: None
        """
    @property
    def incrementalRelocatesSourceToTarget(self) -> None:
        """
        :type: None
        """
    @property
    def incrementalRelocatesTargetToSource(self) -> None:
        """
        :type: None
        """
    @property
    def layerOffsets(self) -> None:
        """
        :type: None
        """
    @property
    def layerTree(self) -> None:
        """
        :type: None
        """
    @property
    def layers(self) -> None:
        """
        :type: None
        """
    @property
    def localErrors(self) -> None:
        """
        :type: None
        """
    @property
    def mutedLayers(self) -> None:
        """
        :type: None
        """
    @property
    def pathsToPrimsWithRelocates(self) -> None:
        """
        :type: None
        """
    @property
    def relocatesSourceToTarget(self) -> None:
        """
        :type: None
        """
    @property
    def relocatesTargetToSource(self) -> None:
        """
        :type: None
        """
    @property
    def sessionLayerTree(self) -> None:
        """
        :type: None
        """
    pass
class LayerStackIdentifier(Boost.Python.instance):
    @property
    def expressionVariablesOverrideSource(self) -> None:
        """
        :type: None
        """
    @property
    def pathResolverContext(self) -> None:
        """
        :type: None
        """
    @property
    def rootLayer(self) -> None:
        """
        :type: None
        """
    @property
    def sessionLayer(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 104
    pass
class LayerStackSite(Boost.Python.instance):
    @property
    def layerStack(self) -> None:
        """
        :type: None
        """
    @property
    def path(self) -> None:
        """
        :type: None
        """
    pass
class MapExpression(Boost.Python.instance):
    @staticmethod
    def AddRootIdentity(*args, **kwargs) -> None: ...
    @staticmethod
    def Compose(*args, **kwargs) -> None: ...
    @staticmethod
    def Constant(*args, **kwargs) -> None: ...
    @staticmethod
    def Evaluate(*args, **kwargs) -> None: ...
    @staticmethod
    def Identity(*args, **kwargs) -> None: ...
    @staticmethod
    def Inverse(*args, **kwargs) -> None: ...
    @staticmethod
    def MapSourceToTarget(*args, **kwargs) -> None: ...
    @staticmethod
    def MapTargetToSource(*args, **kwargs) -> None: ...
    @property
    def isIdentity(self) -> None:
        """
        :type: None
        """
    @property
    def isNull(self) -> None:
        """
        :type: None
        """
    @property
    def timeOffset(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 32
    pass
class MapFunction(Boost.Python.instance):
    @staticmethod
    def Compose(*args, **kwargs) -> None: ...
    @staticmethod
    def ComposeOffset(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def Identity(*args, **kwargs) -> None: ...
    @staticmethod
    def IdentityPathMap(*args, **kwargs) -> None: ...
    @staticmethod
    def MapSourceToTarget(*args, **kwargs) -> None: ...
    @staticmethod
    def MapTargetToSource(*args, **kwargs) -> None: ...
    @property
    def isIdentity(self) -> None:
        """
        :type: None
        """
    @property
    def isIdentityPathMapping(self) -> None:
        """
        :type: None
        """
    @property
    def isNull(self) -> None:
        """
        :type: None
        """
    @property
    def sourceToTargetMap(self) -> None:
        """
        :type: None
        """
    @property
    def timeOffset(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 80
    pass
class NodeRef(Boost.Python.instance):
    @staticmethod
    def CanContributeSpecs(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDepthBelowIntroduction(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIntroPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOriginRootNode(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPathAtIntroduction(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRootNode(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSpecContributionRestrictedDepth(*args, **kwargs) -> None: ...
    @staticmethod
    def IsDueToAncestor(*args, **kwargs) -> None: ...
    @staticmethod
    def IsRootNode(*args, **kwargs) -> None: ...
    @property
    def arcType(self) -> None:
        """
        :type: None
        """
    @property
    def children(self) -> None:
        """
        :type: None
        """
    @property
    def hasSpecs(self) -> None:
        """
        :type: None
        """
    @property
    def hasSymmetry(self) -> None:
        """
        :type: None
        """
    @property
    def isCulled(self) -> None:
        """
        :type: None
        """
    @property
    def isInert(self) -> None:
        """
        :type: None
        """
    @property
    def isRestricted(self) -> None:
        """
        :type: None
        """
    @property
    def layerStack(self) -> None:
        """
        :type: None
        """
    @property
    def mapToParent(self) -> None:
        """
        :type: None
        """
    @property
    def mapToRoot(self) -> None:
        """
        :type: None
        """
    @property
    def namespaceDepth(self) -> None:
        """
        :type: None
        """
    @property
    def origin(self) -> None:
        """
        :type: None
        """
    @property
    def parent(self) -> None:
        """
        :type: None
        """
    @property
    def path(self) -> None:
        """
        :type: None
        """
    @property
    def permission(self) -> None:
        """
        :type: None
        """
    @property
    def siblingNumAtOrigin(self) -> None:
        """
        :type: None
        """
    @property
    def site(self) -> None:
        """
        :type: None
        """
    pass
class PrimIndex(Boost.Python.instance):
    @staticmethod
    def ComposeAuthoredVariantSelections(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputePrimChildNames(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputePrimPropertyNames(*args, **kwargs) -> None: ...
    @staticmethod
    def DumpToDotGraph(*args, **kwargs) -> None: ...
    @staticmethod
    def DumpToString(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeProvidingSpec(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSelectionAppliedForVariantSet(*args, **kwargs) -> None: ...
    @staticmethod
    def IsInstanceable(*args, **kwargs) -> None: ...
    @staticmethod
    def IsUsd(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValid(*args, **kwargs) -> None: ...
    @staticmethod
    def PrintStatistics(*args, **kwargs) -> None: ...
    @property
    def hasAnyPayloads(self) -> None:
        """
        :type: None
        """
    @property
    def localErrors(self) -> None:
        """
        :type: None
        """
    @property
    def primStack(self) -> None:
        """
        :type: None
        """
    @property
    def rootNode(self) -> None:
        """
        :type: None
        """
    pass
class PropertyIndex(Boost.Python.instance):
    @property
    def localErrors(self) -> None:
        """
        :type: None
        """
    @property
    def localPropertyStack(self) -> None:
        """
        :type: None
        """
    @property
    def propertyStack(self) -> None:
        """
        :type: None
        """
    pass
class Site(Boost.Python.instance):
    @property
    def layerStack(self) -> None:
        """
        :type: None
        """
    @property
    def path(self) -> None:
        """
        :type: None
        """
    pass
class _LayerRelocatesEditBuilderRelocateResult(Boost.Python.instance):
    @property
    def whyNot(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
class _TestChangeProcessor(Boost.Python.instance):
    @staticmethod
    def GetPrimChanges(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSignificantChanges(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSpecChanges(*args, **kwargs) -> None: ...
    __instance_size__ = 40
    pass
def BuildPrimPropertyIndex(*args, **kwargs) -> None:
    pass
def TranslatePathFromNodeToRoot(*args, **kwargs) -> None:
    pass
def TranslatePathFromRootToNode(*args, **kwargs) -> None:
    pass
def _GetInvalidPcpNode(*args, **kwargs) -> None:
    pass
ArcTypeInherit: pxr.Pcp.ArcType # value = Pcp.ArcTypeInherit
ArcTypePayload: pxr.Pcp.ArcType # value = Pcp.ArcTypePayload
ArcTypeReference: pxr.Pcp.ArcType # value = Pcp.ArcTypeReference
ArcTypeRelocate: pxr.Pcp.ArcType # value = Pcp.ArcTypeRelocate
ArcTypeRoot: pxr.Pcp.ArcType # value = Pcp.ArcTypeRoot
ArcTypeSpecialize: pxr.Pcp.ArcType # value = Pcp.ArcTypeSpecialize
ArcTypeVariant: pxr.Pcp.ArcType # value = Pcp.ArcTypeVariant
DependencyTypeAncestral: pxr.Pcp.DependencyType # value = Pcp.DependencyTypeAncestral
DependencyTypeAnyIncludingVirtual: pxr.Pcp.DependencyType # value = Pcp.DependencyTypeAnyIncludingVirtual
DependencyTypeAnyNonVirtual: pxr.Pcp.DependencyType # value = Pcp.DependencyTypeAnyNonVirtual
DependencyTypeDirect: pxr.Pcp.DependencyType # value = Pcp.DependencyTypeDirect
DependencyTypeNonVirtual: pxr.Pcp.DependencyType # value = Pcp.DependencyTypeNonVirtual
DependencyTypeNone: pxr.Pcp.DependencyType # value = Pcp.DependencyTypeNone
DependencyTypePartlyDirect: pxr.Pcp.DependencyType # value = Pcp.DependencyTypePartlyDirect
DependencyTypePurelyDirect: pxr.Pcp.DependencyType # value = Pcp.DependencyTypePurelyDirect
DependencyTypeRoot: pxr.Pcp.DependencyType # value = Pcp.DependencyTypeRoot
DependencyTypeVirtual: pxr.Pcp.DependencyType # value = Pcp.DependencyTypeVirtual
ErrorType_ArcCapacityExceeded: pxr.Pcp.ErrorType # value = Pcp.ErrorType_ArcCapacityExceeded
ErrorType_ArcCycle: pxr.Pcp.ErrorType # value = Pcp.ErrorType_ArcCycle
ErrorType_ArcNamespaceDepthCapacityExceeded: pxr.Pcp.ErrorType # value = Pcp.ErrorType_ArcNamespaceDepthCapacityExceeded
ErrorType_ArcPermissionDenied: pxr.Pcp.ErrorType # value = Pcp.ErrorType_ArcPermissionDenied
ErrorType_InconsistentAttributeType: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InconsistentAttributeType
ErrorType_InconsistentAttributeVariability: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InconsistentAttributeVariability
ErrorType_InconsistentPropertyType: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InconsistentPropertyType
ErrorType_IndexCapacityExceeded: pxr.Pcp.ErrorType # value = Pcp.ErrorType_IndexCapacityExceeded
ErrorType_InternalAssetPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InternalAssetPath
ErrorType_InvalidAssetPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidAssetPath
ErrorType_InvalidAuthoredRelocation: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidAuthoredRelocation
ErrorType_InvalidConflictingRelocation: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidConflictingRelocation
ErrorType_InvalidExternalTargetPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidExternalTargetPath
ErrorType_InvalidInstanceTargetPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidInstanceTargetPath
ErrorType_InvalidPrimPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidPrimPath
ErrorType_InvalidReferenceOffset: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidReferenceOffset
ErrorType_InvalidSameTargetRelocations: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidSameTargetRelocations
ErrorType_InvalidSublayerOffset: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidSublayerOffset
ErrorType_InvalidSublayerOwnership: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidSublayerOwnership
ErrorType_InvalidSublayerPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidSublayerPath
ErrorType_InvalidTargetPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidTargetPath
ErrorType_InvalidVariantSelection: pxr.Pcp.ErrorType # value = Pcp.ErrorType_InvalidVariantSelection
ErrorType_MutedAssetPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_MutedAssetPath
ErrorType_OpinionAtRelocationSource: pxr.Pcp.ErrorType # value = Pcp.ErrorType_OpinionAtRelocationSource
ErrorType_PrimPermissionDenied: pxr.Pcp.ErrorType # value = Pcp.ErrorType_PrimPermissionDenied
ErrorType_PropertyPermissionDenied: pxr.Pcp.ErrorType # value = Pcp.ErrorType_PropertyPermissionDenied
ErrorType_SublayerCycle: pxr.Pcp.ErrorType # value = Pcp.ErrorType_SublayerCycle
ErrorType_TargetPermissionDenied: pxr.Pcp.ErrorType # value = Pcp.ErrorType_TargetPermissionDenied
ErrorType_UnresolvedPrimPath: pxr.Pcp.ErrorType # value = Pcp.ErrorType_UnresolvedPrimPath
ErrorType_VariableExpressionError: pxr.Pcp.ErrorType # value = Pcp.ErrorType_VariableExpressionError
__MFB_FULL_PACKAGE_NAME = 'pcp'
