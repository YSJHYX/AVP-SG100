from __future__ import annotations
import pxr.Sdf._sdf
import typing
import Boost.Python
import pxr.Sdf
import pxr.Tf

__all__ = [
    "AngularUnit",
    "AngularUnitDegrees",
    "AngularUnitRadians",
    "AssetPath",
    "AssetPathArray",
    "AttributeSpec",
    "AuthoringError",
    "AuthoringErrorUnrecognizedFields",
    "AuthoringErrorUnrecognizedSpecType",
    "BatchNamespaceEdit",
    "ChangeBlock",
    "ChildrenView_Sdf_AttributeChildPolicy_SdfAttributeViewPredicate",
    "ChildrenView_Sdf_AttributeChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfAttributeSpec___",
    "ChildrenView_Sdf_PrimChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPrimSpec___",
    "ChildrenView_Sdf_PropertyChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPropertySpec___",
    "ChildrenView_Sdf_RelationshipChildPolicy_SdfRelationshipViewPredicate",
    "ChildrenView_Sdf_VariantChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSpec___",
    "ChildrenView_Sdf_VariantSetChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSetSpec___",
    "CleanupEnabler",
    "ComputeAssetPathRelativeToLayer",
    "ConvertToValidMetadataDictionary",
    "ConvertUnit",
    "CopySpec",
    "CreatePrimAttributeInLayer",
    "CreatePrimInLayer",
    "CreateRelationshipInLayer",
    "CreateVariantInLayer",
    "DefaultUnit",
    "DimensionlessUnit",
    "DimensionlessUnitDefault",
    "DimensionlessUnitPercent",
    "FileFormat",
    "GetNameForUnit",
    "GetTypeForValueTypeName",
    "GetUnitFromName",
    "GetValueTypeNameForValue",
    "Int64ListOp",
    "IntListOp",
    "JustCreatePrimAttributeInLayer",
    "JustCreatePrimInLayer",
    "JustCreateRelationshipInLayer",
    "Layer",
    "LayerOffset",
    "LayerTree",
    "LengthUnit",
    "LengthUnitCentimeter",
    "LengthUnitDecimeter",
    "LengthUnitFoot",
    "LengthUnitInch",
    "LengthUnitKilometer",
    "LengthUnitMeter",
    "LengthUnitMile",
    "LengthUnitMillimeter",
    "LengthUnitYard",
    "ListEditorProxy_SdfNameKeyPolicy",
    "ListEditorProxy_SdfPathKeyPolicy",
    "ListEditorProxy_SdfPayloadTypePolicy",
    "ListEditorProxy_SdfReferenceTypePolicy",
    "ListOpType",
    "ListOpTypeAdded",
    "ListOpTypeAppended",
    "ListOpTypeDeleted",
    "ListOpTypeExplicit",
    "ListOpTypeOrdered",
    "ListOpTypePrepended",
    "ListProxy_SdfNameKeyPolicy",
    "ListProxy_SdfNameTokenKeyPolicy",
    "ListProxy_SdfPathKeyPolicy",
    "ListProxy_SdfPayloadTypePolicy",
    "ListProxy_SdfReferenceTypePolicy",
    "ListProxy_SdfSubLayerTypePolicy",
    "MapEditProxy_VtDictionary",
    "MapEditProxy_map_SdfPath__SdfPath__less_SdfPath___allocator_pair_SdfPath_const__SdfPath_____",
    "MapEditProxy_map_string__string__less_string___allocator_pair_stringconst__string_____",
    "NamespaceEdit",
    "NamespaceEditDetail",
    "Notice",
    "OpaqueValue",
    "Path",
    "PathArray",
    "PathExpression",
    "PathExpressionArray",
    "PathListOp",
    "PathPattern",
    "Payload",
    "PayloadListOp",
    "Permission",
    "PermissionPrivate",
    "PermissionPublic",
    "PredicateExpression",
    "PredicateFunctionResult",
    "PrimSpec",
    "PropertySpec",
    "PseudoRootSpec",
    "Reference",
    "ReferenceListOp",
    "RelationshipSpec",
    "Spec",
    "SpecType",
    "SpecTypeAttribute",
    "SpecTypeConnection",
    "SpecTypeExpression",
    "SpecTypeMapper",
    "SpecTypeMapperArg",
    "SpecTypePrim",
    "SpecTypePseudoRoot",
    "SpecTypeRelationship",
    "SpecTypeRelationshipTarget",
    "SpecTypeUnknown",
    "SpecTypeVariant",
    "SpecTypeVariantSet",
    "Specifier",
    "SpecifierClass",
    "SpecifierDef",
    "SpecifierOver",
    "StringListOp",
    "TimeCode",
    "TimeCodeArray",
    "TokenListOp",
    "UInt64ListOp",
    "UIntListOp",
    "UnitCategory",
    "UnregisteredValue",
    "UnregisteredValueListOp",
    "ValueBlock",
    "ValueHasValidType",
    "ValueRoleNames",
    "ValueTypeName",
    "ValueTypeNames",
    "Variability",
    "VariabilityUniform",
    "VariabilityVarying",
    "VariableExpression",
    "VariantSetSpec",
    "VariantSpec"
]


class AngularUnit(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.AngularUnitDegrees, Sdf.AngularUnitRadians)
    pass
class AssetPath(Boost.Python.instance):
    @property
    def authoredPath(self) -> None:
        """
        :type: None
        """
    @property
    def evaluatedPath(self) -> None:
        """
        :type: None
        """
    @property
    def path(self) -> None:
        """
        :type: None
        """
    @property
    def resolvedPath(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 120
    pass
class AssetPathArray(Boost.Python.instance):
    """
    An array of type SdfAssetPath.
    """
    _isVtArray = True
    pass
class AttributeSpec(PropertySpec, Spec, Boost.Python.instance):
    @staticmethod
    def ClearColorSpace(*args, **kwargs) -> None: ...
    @staticmethod
    def EraseTimeSample(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBracketingTimeSamples(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNumTimeSamples(*args, **kwargs) -> None: ...
    @staticmethod
    def HasColorSpace(*args, **kwargs) -> None: ...
    @staticmethod
    def ListTimeSamples(*args, **kwargs) -> None: ...
    @staticmethod
    def QueryTimeSample(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTimeSample(*args, **kwargs) -> None: ...
    @property
    def allowedTokens(self) -> None:
        """
        The allowed value tokens for this property

        :type: None
        """
    @property
    def colorSpace(self) -> None:
        """
        The color-space in which the attribute value is authored.

        :type: None
        """
    @property
    def connectionPathList(self) -> None:
        """
        A PathListEditor for the attribute's connection paths.

        The list of the connection paths for this attribute may be modified with this PathListEditor.

        A PathListEditor may express a list either as an explicit value or as a set of list editing operations.  See GdListEditor for more information.

        :type: None
        """
    @property
    def displayUnit(self) -> None:
        """
        The display unit for this attribute.

        :type: None
        """
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    @property
    def roleName(self) -> None:
        """
        The roleName for this attribute's typeName.

        :type: None
        """
    @property
    def typeName(self) -> None:
        """
        The typename of this attribute.

        :type: None
        """
    @property
    def valueType(self) -> None:
        """
        The value type of this attribute.

        :type: None
        """
    ConnectionPathsKey = 'connectionPaths'
    DefaultValueKey = 'default'
    DisplayUnitKey = 'displayUnit'
    pass
class AuthoringError(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.AuthoringErrorUnrecognizedFields, Sdf.AuthoringErrorUnrecognizedSpecType)
    pass
class BatchNamespaceEdit(Boost.Python.instance):
    @staticmethod
    def Add(*args, **kwargs) -> None: ...
    @staticmethod
    def Process(*args, **kwargs) -> None: ...
    @property
    def edits(self) -> None:
        """
        :type: None
        """
    pass
class ChangeBlock(Boost.Python.instance):
    __instance_size__ = 40
    pass
class ChildrenView_Sdf_AttributeChildPolicy_SdfAttributeViewPredicate(Boost.Python.instance):
    class ChildrenView_Sdf_AttributeChildPolicy_SdfAttributeViewPredicate_Iterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_AttributeChildPolicy_SdfAttributeViewPredicate_KeyIterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_AttributeChildPolicy_SdfAttributeViewPredicate_ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    pass
class ChildrenView_Sdf_AttributeChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfAttributeSpec___(Boost.Python.instance):
    class ChildrenView_Sdf_AttributeChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfAttributeSpec____Iterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_AttributeChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfAttributeSpec____KeyIterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_AttributeChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfAttributeSpec____ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    pass
class ChildrenView_Sdf_PrimChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPrimSpec___(Boost.Python.instance):
    class ChildrenView_Sdf_PrimChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPrimSpec____Iterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_PrimChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPrimSpec____KeyIterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_PrimChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPrimSpec____ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    pass
class ChildrenView_Sdf_PropertyChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPropertySpec___(Boost.Python.instance):
    class ChildrenView_Sdf_PropertyChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPropertySpec____Iterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_PropertyChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPropertySpec____KeyIterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_PropertyChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfPropertySpec____ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    pass
class ChildrenView_Sdf_RelationshipChildPolicy_SdfRelationshipViewPredicate(Boost.Python.instance):
    class ChildrenView_Sdf_RelationshipChildPolicy_SdfRelationshipViewPredicate_Iterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_RelationshipChildPolicy_SdfRelationshipViewPredicate_KeyIterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_RelationshipChildPolicy_SdfRelationshipViewPredicate_ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    pass
class ChildrenView_Sdf_VariantChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSpec___(Boost.Python.instance):
    class ChildrenView_Sdf_VariantChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSpec____Iterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_VariantChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSpec____KeyIterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_VariantChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSpec____ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    pass
class ChildrenView_Sdf_VariantSetChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSetSpec___(Boost.Python.instance):
    class ChildrenView_Sdf_VariantSetChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSetSpec____Iterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_VariantSetChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSetSpec____KeyIterator(Boost.Python.instance):
        pass
    class ChildrenView_Sdf_VariantSetChildPolicy_SdfChildrenViewTrivialPredicate_SdfHandle_SdfVariantSetSpec____ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    pass
class CleanupEnabler(Boost.Python.instance):
    __instance_size__ = 32
    pass
class DimensionlessUnit(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.DimensionlessUnitPercent, Sdf.DimensionlessUnitDefault)
    pass
class FileFormat(Boost.Python.instance):
    class Tokens(Boost.Python.instance):
        TargetArg = 'target'
        pass
    @staticmethod
    def CanRead(*args, **kwargs) -> None: ...
    @staticmethod
    def FindAllDerivedFileFormatExtensions(*args, **kwargs) -> None: ...
    @staticmethod
    def FindAllFileFormatExtensions(*args, **kwargs) -> None: ...
    @staticmethod
    def FindByExtension(*args, **kwargs) -> None: ...
    @staticmethod
    def FindById(*args, **kwargs) -> None: ...
    @staticmethod
    def FormatSupportsEditing(*args, **kwargs) -> None: ...
    @staticmethod
    def FormatSupportsReading(*args, **kwargs) -> None: ...
    @staticmethod
    def FormatSupportsWriting(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFileExtension(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFileExtensions(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPackage(*args, **kwargs) -> None: ...
    @staticmethod
    def IsSupportedExtension(*args, **kwargs) -> None: ...
    @staticmethod
    def SupportsEditing(*args, **kwargs) -> None: ...
    @staticmethod
    def SupportsReading(*args, **kwargs) -> None: ...
    @staticmethod
    def SupportsWriting(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    @property
    def fileCookie(self) -> None:
        """
        :type: None
        """
    @property
    def formatId(self) -> None:
        """
        :type: None
        """
    @property
    def primaryFileExtension(self) -> None:
        """
        :type: None
        """
    @property
    def target(self) -> None:
        """
        :type: None
        """
    pass
class Int64ListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class IntListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class Layer(Boost.Python.instance):
    class DetachedLayerRules(Boost.Python.instance):
        @staticmethod
        def Exclude(*args, **kwargs) -> None: ...
        @staticmethod
        def GetExcluded(*args, **kwargs) -> None: ...
        @staticmethod
        def GetIncluded(*args, **kwargs) -> None: ...
        @staticmethod
        def Include(*args, **kwargs) -> None: ...
        @staticmethod
        def IncludeAll(*args, **kwargs) -> None: ...
        @staticmethod
        def IncludedAll(*args, **kwargs) -> None: ...
        @staticmethod
        def IsIncluded(*args, **kwargs) -> None: ...
        __instance_size__ = 80
        pass
    @staticmethod
    def AddToMutedLayers(*args, **kwargs) -> None: ...
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyRootPrimOrder(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearColorConfiguration(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearColorManagementSystem(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearCustomLayerData(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearDefaultPrim(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEndTimeCode(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearExpressionVariables(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearFramePrecision(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearFramesPerSecond(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearOwner(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearRelocates(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearSessionOwner(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearStartTimeCode(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearTimeCodesPerSecond(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeAbsolutePath(*args, **kwargs) -> None: ...
    @staticmethod
    def ConvertDefaultPrimPathToToken(*args, **kwargs) -> None: ...
    @staticmethod
    def ConvertDefaultPrimTokenToPath(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateAnonymous(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateNew(*args, **kwargs) -> None: ...
    @staticmethod
    def DumpLayerInfo(*args, **kwargs) -> None: 
        """
        Debug helper to examine content of the current layer registry and
        the asset/real path of all layers in the registry.
        """
    @staticmethod
    def EraseTimeSample(*args, **kwargs) -> None: ...
    @staticmethod
    def Export(*args, **kwargs) -> None: ...
    @staticmethod
    def ExportToString(*args, **kwargs) -> None: 
        """
        Returns the string representation of the layer.
        """
    @staticmethod
    def Find(filename) -> LayerPtr: 
        """
        filename : string

        Returns the open layer with the given filename, or None.  Note that this is a static class method.
        """
    @staticmethod
    def FindOrOpen(*args, **kwargs) -> None: ...
    @staticmethod
    def FindOrOpenRelativeToLayer(*args, **kwargs) -> None: ...
    @staticmethod
    def FindRelativeToLayer(*args, **kwargs) -> None: 
        """
        Returns the open layer with the given filename, or None.  If the filename is a relative path then it's found relative to the given layer.  Note that this is a static class method.
        """
    @staticmethod
    def GetAssetInfo(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAssetName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAttributeAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBracketingTimeSamples(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBracketingTimeSamplesForPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCompositionAssetDependencies(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDefaultPrimAsPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDetachedLayerRules(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDisplayName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDisplayNameFromIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def GetExternalAssetDependencies(*args, **kwargs) -> None: ...
    @staticmethod
    def GetExternalReferences(*args, **kwargs) -> None: 
        """
        Return a list of asset paths for
        this layer.
        """
    @staticmethod
    def GetFileFormat(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFileFormatArguments(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLoadedLayers(*args, **kwargs) -> None: 
        """
        Return list of loaded layers.
        """
    @staticmethod
    def GetMutedLayers(*args, **kwargs) -> None: 
        """
        Return list of muted layers.
        """
    @staticmethod
    def GetNumTimeSamplesForPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetObjectAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPreviousTimeSampleForPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrimAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPropertyAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRelationshipAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def HasColorConfiguration(*args, **kwargs) -> None: ...
    @staticmethod
    def HasColorManagementSystem(*args, **kwargs) -> None: ...
    @staticmethod
    def HasCustomLayerData(*args, **kwargs) -> None: ...
    @staticmethod
    def HasDefaultPrim(*args, **kwargs) -> None: ...
    @staticmethod
    def HasEndTimeCode(*args, **kwargs) -> None: ...
    @staticmethod
    def HasExpressionVariables(*args, **kwargs) -> None: ...
    @staticmethod
    def HasFramePrecision(*args, **kwargs) -> None: ...
    @staticmethod
    def HasFramesPerSecond(*args, **kwargs) -> None: ...
    @staticmethod
    def HasOwner(*args, **kwargs) -> None: ...
    @staticmethod
    def HasRelocates(*args, **kwargs) -> None: ...
    @staticmethod
    def HasSessionOwner(*args, **kwargs) -> None: ...
    @staticmethod
    def HasStartTimeCode(*args, **kwargs) -> None: ...
    @staticmethod
    def HasTimeCodesPerSecond(*args, **kwargs) -> None: ...
    @staticmethod
    def Import(*args, **kwargs) -> None: ...
    @staticmethod
    def ImportFromString(*args, **kwargs) -> None: ...
    @staticmethod
    def IsAnonymousLayerIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def IsDetached(*args, **kwargs) -> None: ...
    @staticmethod
    def IsIncludedByDetachedLayerRules(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMuted(*args, **kwargs) -> None: ...
    @staticmethod
    def ListAllTimeSamples(*args, **kwargs) -> None: ...
    @staticmethod
    def ListTimeSamplesForPath(*args, **kwargs) -> None: ...
    @staticmethod
    def New(*args, **kwargs) -> None: ...
    @staticmethod
    def OpenAsAnonymous(*args, **kwargs) -> None: ...
    @staticmethod
    def QueryTimeSample(*args, **kwargs) -> None: ...
    @staticmethod
    def Reload(*args, **kwargs) -> None: ...
    @staticmethod
    def ReloadLayers(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveFromMutedLayers(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveInertSceneDescription(*args, **kwargs) -> None: ...
    @staticmethod
    def RemovePropertyIfHasOnlyRequiredFields(*args, **kwargs) -> None: ...
    @staticmethod
    def Save(*args, **kwargs) -> None: ...
    @staticmethod
    def ScheduleRemoveIfInert(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDetachedLayerRules(*args, **kwargs) -> None: ...
    @staticmethod
    def SetMuted(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPermissionToEdit(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPermissionToSave(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTimeSample(*args, **kwargs) -> None: ...
    @staticmethod
    def SplitIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def StreamsData(*args, **kwargs) -> None: ...
    @staticmethod
    def TransferContent(*args, **kwargs) -> None: ...
    @staticmethod
    def Traverse(*args, **kwargs) -> None: ...
    @staticmethod
    def UpdateAssetInfo(*args, **kwargs) -> None: ...
    @staticmethod
    def UpdateCompositionAssetDependency(*args, **kwargs) -> None: ...
    @staticmethod
    def UpdateExternalReference(*args, **kwargs) -> None: ...
    @staticmethod
    def _WriteDataFile(*args, **kwargs) -> None: ...
    @property
    def anonymous(self) -> None:
        """
        :type: None
        """
    @property
    def colorConfiguration(self) -> None:
        """
        The color configuration asset-path of this layer.

        :type: None
        """
    @property
    def colorManagementSystem(self) -> None:
        """
        The name of the color management system used to interpret the colorConfiguration asset.

        :type: None
        """
    @property
    def comment(self) -> None:
        """
        The layer's comment string.

        :type: None
        """
    @property
    def customLayerData(self) -> None:
        """
        The customLayerData dictionary associated with this layer.

        :type: None
        """
    @property
    def defaultPrim(self) -> None:
        """
        The layer's default reference target token.

        :type: None
        """
    @property
    def dirty(self) -> None:
        """
        :type: None
        """
    @property
    def documentation(self) -> None:
        """
        The layer's documentation string.

        :type: None
        """
    @property
    def empty(self) -> None:
        """
        :type: None
        """
    @property
    def endTimeCode(self) -> None:
        """
        The end timeCode of this layer.

        The end timeCode of a layer is not a hard limit, but is 
        more of a hint. A layer's time-varying content is not limited to
        the timeCode range of the layer.

        :type: None
        """
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    @property
    def expressionVariables(self) -> None:
        """
        The expressionVariables dictionary associated with this layer.

        :type: None
        """
    @property
    def externalReferences(self) -> None:
        """
        Return unique list of asset paths of external references for
        given layer.

        :type: None
        """
    @property
    def fileExtension(self) -> None:
        """
        The layer's file extension.

        :type: None
        """
    @property
    def framePrecision(self) -> None:
        """
        The number of digits of precision used in times in this layer.

        :type: None
        """
    @property
    def framesPerSecond(self) -> None:
        """
        The frames per second used in this layer.

        :type: None
        """
    @property
    def hasOwnedSubLayers(self) -> None:
        """
        Whether this layer's sub layers are expected to have owners.

        :type: None
        """
    @property
    def identifier(self) -> None:
        """
        The layer's identifier.

        :type: None
        """
    @property
    def owner(self) -> None:
        """
        The owner of this layer.

        :type: None
        """
    @property
    def permissionToEdit(self) -> None:
        """
        Return true if permitted to be edited (modified), false otherwise.

        :type: None
        """
    @property
    def permissionToSave(self) -> None:
        """
        Return true if permitted to be saved, false otherwise.

        :type: None
        """
    @property
    def pseudoRoot(self) -> None:
        """
        The pseudo-root of the layer.

        :type: None
        """
    @property
    def realPath(self) -> None:
        """
        The layer's resolved path.

        :type: None
        """
    @property
    def relocates(self) -> None:
        """
        :type: None
        """
    @property
    def repositoryPath(self) -> None:
        """
        The layer's associated repository path

        :type: None
        """
    @property
    def resolvedPath(self) -> None:
        """
        The layer's resolved path.

        :type: None
        """
    @property
    def rootPrimOrder(self) -> None:
        """
        Get/set the list of root prim names for this layer's 'reorder rootPrims' statement.

        :type: None
        """
    @property
    def rootPrims(self) -> None:
        """
        The root prims of this layer, as an ordered dictionary.

        The prims may be accessed by index or by name.
        Although this property claims it is read only, you can modify the contents of this dictionary to add, remove, or reorder the contents.

        :type: None
        """
    @property
    def sessionOwner(self) -> None:
        """
        The session owner of this layer. Only intended for use with session layers.

        :type: None
        """
    @property
    def startTimeCode(self) -> None:
        """
        The start timeCode of this layer.

        The start timeCode of a layer is not a hard limit, but is 
        more of a hint.  A layer's time-varying content is not limited to 
        the timeCode range of the layer.

        :type: None
        """
    @property
    def subLayerOffsets(self) -> None:
        """
        The sublayer offsets of this layer, as a list.  Although this property is claimed to be read only, you can modify the contents of this list by assigning new layer offsets to specific indices.

        :type: None
        """
    @property
    def subLayerPaths(self) -> None:
        """
        The sublayer paths of this layer, as a list.  Although this property is claimed to be read only, you can modify the contents of this list.

        :type: None
        """
    @property
    def timeCodesPerSecond(self) -> None:
        """
        The timeCodes per second used in this layer.

        :type: None
        """
    @property
    def version(self) -> None:
        """
        The layer's version.

        :type: None
        """
    ColorConfigurationKey = 'colorConfiguration'
    ColorManagementSystemKey = 'colorManagementSystem'
    CommentKey = 'comment'
    DocumentationKey = 'documentation'
    EndFrameKey = 'endFrame'
    EndTimeCodeKey = 'endTimeCode'
    FramePrecisionKey = 'framePrecision'
    FramesPerSecondKey = 'framesPerSecond'
    HasOwnedSubLayers = 'hasOwnedSubLayers'
    LayerRelocatesKey = 'layerRelocates'
    OwnerKey = 'owner'
    SessionOwnerKey = 'sessionOwner'
    StartFrameKey = 'startFrame'
    StartTimeCodeKey = 'startTimeCode'
    TimeCodesPerSecondKey = 'timeCodesPerSecond'
    pass
class LayerOffset(Boost.Python.instance):
    @staticmethod
    def GetInverse(*args, **kwargs) -> None: ...
    @staticmethod
    def IsIdentity(*args, **kwargs) -> None: ...
    @property
    def offset(self) -> None:
        """
        :type: None
        """
    @property
    def scale(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class LayerTree(Boost.Python.instance):
    @property
    def childTrees(self) -> None:
        """
        :type: None
        """
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    @property
    def layer(self) -> None:
        """
        :type: None
        """
    @property
    def offset(self) -> None:
        """
        :type: None
        """
    pass
class LengthUnit(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.LengthUnitMillimeter, Sdf.LengthUnitCentimeter, Sdf.LengthUnitDecimeter, Sdf.LengthUnitMeter, Sdf.LengthUnitKilometer, Sdf.LengthUnitInch, Sdf.LengthUnitFoot, Sdf.LengthUnitYard, Sdf.LengthUnitMile)
    pass
class ListEditorProxy_SdfNameKeyPolicy(Boost.Python.instance):
    @staticmethod
    def Add(*args, **kwargs) -> None: ...
    @staticmethod
    def Append(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEditsAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsItemEdit(*args, **kwargs) -> None: ...
    @staticmethod
    def CopyItems(*args, **kwargs) -> None: ...
    @staticmethod
    def Erase(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def ModifyItemEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def Prepend(*args, **kwargs) -> None: ...
    @staticmethod
    def Remove(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveItemEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplaceItemEdits(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExpired(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def isOrderedOnly(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    pass
class ListEditorProxy_SdfPathKeyPolicy(Boost.Python.instance):
    @staticmethod
    def Add(*args, **kwargs) -> None: ...
    @staticmethod
    def Append(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEditsAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsItemEdit(*args, **kwargs) -> None: ...
    @staticmethod
    def CopyItems(*args, **kwargs) -> None: ...
    @staticmethod
    def Erase(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def ModifyItemEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def Prepend(*args, **kwargs) -> None: ...
    @staticmethod
    def Remove(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveItemEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplaceItemEdits(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExpired(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def isOrderedOnly(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    pass
class ListEditorProxy_SdfPayloadTypePolicy(Boost.Python.instance):
    @staticmethod
    def Add(*args, **kwargs) -> None: ...
    @staticmethod
    def Append(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEditsAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsItemEdit(*args, **kwargs) -> None: ...
    @staticmethod
    def CopyItems(*args, **kwargs) -> None: ...
    @staticmethod
    def Erase(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def ModifyItemEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def Prepend(*args, **kwargs) -> None: ...
    @staticmethod
    def Remove(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveItemEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplaceItemEdits(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExpired(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def isOrderedOnly(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    pass
class ListEditorProxy_SdfReferenceTypePolicy(Boost.Python.instance):
    @staticmethod
    def Add(*args, **kwargs) -> None: ...
    @staticmethod
    def Append(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearEditsAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsItemEdit(*args, **kwargs) -> None: ...
    @staticmethod
    def CopyItems(*args, **kwargs) -> None: ...
    @staticmethod
    def Erase(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def ModifyItemEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def Prepend(*args, **kwargs) -> None: ...
    @staticmethod
    def Remove(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveItemEdits(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplaceItemEdits(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExpired(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def isOrderedOnly(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    pass
class ListOpType(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.ListOpTypeExplicit, Sdf.ListOpTypeAdded, Sdf.ListOpTypePrepended, Sdf.ListOpTypeAppended, Sdf.ListOpTypeDeleted, Sdf.ListOpTypeOrdered)
    pass
class ListProxy_SdfNameKeyPolicy(Boost.Python.instance):
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyList(*args, **kwargs) -> None: ...
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def count(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def insert(*args, **kwargs) -> None: ...
    @staticmethod
    def remove(*args, **kwargs) -> None: ...
    @staticmethod
    def replace(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    invalidIndex = -1
    pass
class ListProxy_SdfNameTokenKeyPolicy(Boost.Python.instance):
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyList(*args, **kwargs) -> None: ...
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def count(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def insert(*args, **kwargs) -> None: ...
    @staticmethod
    def remove(*args, **kwargs) -> None: ...
    @staticmethod
    def replace(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    invalidIndex = -1
    pass
class ListProxy_SdfPathKeyPolicy(Boost.Python.instance):
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyList(*args, **kwargs) -> None: ...
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def count(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def insert(*args, **kwargs) -> None: ...
    @staticmethod
    def remove(*args, **kwargs) -> None: ...
    @staticmethod
    def replace(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    invalidIndex = -1
    pass
class ListProxy_SdfPayloadTypePolicy(Boost.Python.instance):
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyList(*args, **kwargs) -> None: ...
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def count(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def insert(*args, **kwargs) -> None: ...
    @staticmethod
    def remove(*args, **kwargs) -> None: ...
    @staticmethod
    def replace(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    invalidIndex = -1
    pass
class ListProxy_SdfReferenceTypePolicy(Boost.Python.instance):
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyList(*args, **kwargs) -> None: ...
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def count(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def insert(*args, **kwargs) -> None: ...
    @staticmethod
    def remove(*args, **kwargs) -> None: ...
    @staticmethod
    def replace(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    invalidIndex = -1
    pass
class ListProxy_SdfSubLayerTypePolicy(Boost.Python.instance):
    @staticmethod
    def ApplyEditsToList(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyList(*args, **kwargs) -> None: ...
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def count(*args, **kwargs) -> None: ...
    @staticmethod
    def index(*args, **kwargs) -> None: ...
    @staticmethod
    def insert(*args, **kwargs) -> None: ...
    @staticmethod
    def remove(*args, **kwargs) -> None: ...
    @staticmethod
    def replace(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    invalidIndex = -1
    pass
class MapEditProxy_VtDictionary(Boost.Python.instance):
    class MapEditProxy_VtDictionary_Iterator(Boost.Python.instance):
        pass
    class MapEditProxy_VtDictionary_KeyIterator(Boost.Python.instance):
        pass
    class MapEditProxy_VtDictionary_ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def pop(*args, **kwargs) -> None: ...
    @staticmethod
    def popitem(*args, **kwargs) -> None: ...
    @staticmethod
    def setdefault(*args, **kwargs) -> None: ...
    @staticmethod
    def update(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class MapEditProxy_map_SdfPath__SdfPath__less_SdfPath___allocator_pair_SdfPath_const__SdfPath_____(Boost.Python.instance):
    class MapEditProxy_map_SdfPath__SdfPath__less_SdfPath___allocator_pair_SdfPath_const__SdfPath______Iterator(Boost.Python.instance):
        pass
    class MapEditProxy_map_SdfPath__SdfPath__less_SdfPath___allocator_pair_SdfPath_const__SdfPath______KeyIterator(Boost.Python.instance):
        pass
    class MapEditProxy_map_SdfPath__SdfPath__less_SdfPath___allocator_pair_SdfPath_const__SdfPath______ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def pop(*args, **kwargs) -> None: ...
    @staticmethod
    def popitem(*args, **kwargs) -> None: ...
    @staticmethod
    def setdefault(*args, **kwargs) -> None: ...
    @staticmethod
    def update(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class MapEditProxy_map_string__string__less_string___allocator_pair_stringconst__string_____(Boost.Python.instance):
    class MapEditProxy_map_string__string__less_string___allocator_pair_stringconst__string______Iterator(Boost.Python.instance):
        pass
    class MapEditProxy_map_string__string__less_string___allocator_pair_stringconst__string______KeyIterator(Boost.Python.instance):
        pass
    class MapEditProxy_map_string__string__less_string___allocator_pair_stringconst__string______ValueIterator(Boost.Python.instance):
        pass
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def copy(*args, **kwargs) -> None: ...
    @staticmethod
    def get(*args, **kwargs) -> None: ...
    @staticmethod
    def items(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def pop(*args, **kwargs) -> None: ...
    @staticmethod
    def popitem(*args, **kwargs) -> None: ...
    @staticmethod
    def setdefault(*args, **kwargs) -> None: ...
    @staticmethod
    def update(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class NamespaceEdit(Boost.Python.instance):
    @staticmethod
    def Remove(*args, **kwargs) -> None: ...
    @staticmethod
    def Rename(*args, **kwargs) -> None: ...
    @staticmethod
    def Reorder(*args, **kwargs) -> None: ...
    @staticmethod
    def Reparent(*args, **kwargs) -> None: ...
    @staticmethod
    def ReparentAndRename(*args, **kwargs) -> None: ...
    @property
    def currentPath(self) -> None:
        """
        :type: None
        """
    @property
    def index(self) -> None:
        """
        :type: None
        """
    @property
    def newPath(self) -> None:
        """
        :type: None
        """
    atEnd = -1
    same = -2
    pass
class NamespaceEditDetail(Boost.Python.instance):
    class Result(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'NamespaceEditDetail'
        allValues: tuple # value = (Sdf.NamespaceEditDetail.Error, Sdf.NamespaceEditDetail.Unbatched, Sdf.NamespaceEditDetail.Okay)
        pass
    @property
    def edit(self) -> None:
        """
        :type: None
        """
    @property
    def reason(self) -> None:
        """
        :type: None
        """
    @property
    def result(self) -> None:
        """
        :type: None
        """
    Error: pxr.Sdf.Result # value = Sdf.NamespaceEditDetail.Error
    Okay: pxr.Sdf.Result # value = Sdf.NamespaceEditDetail.Okay
    Unbatched: pxr.Sdf.Result # value = Sdf.NamespaceEditDetail.Unbatched
    pass
class Notice(Boost.Python.instance):
    class Base(pxr.Tf.Notice, Boost.Python.instance):
        pass
    class LayerDidReloadContent(LayerDidReplaceContent, Base, pxr.Tf.Notice, Boost.Python.instance):
        pass
    class LayerDidReplaceContent(Base, pxr.Tf.Notice, Boost.Python.instance):
        pass
    class LayerDirtinessChanged(Base, pxr.Tf.Notice, Boost.Python.instance):
        pass
    class LayerIdentifierDidChange(Base, pxr.Tf.Notice, Boost.Python.instance):
        @property
        def newIdentifier(self) -> None:
            """
            :type: None
            """
        @property
        def oldIdentifier(self) -> None:
            """
            :type: None
            """
        pass
    class LayerInfoDidChange(Base, pxr.Tf.Notice, Boost.Python.instance):
        @staticmethod
        def key(*args, **kwargs) -> None: ...
        pass
    class LayerMutenessChanged(Base, pxr.Tf.Notice, Boost.Python.instance):
        @property
        def layerPath(self) -> None:
            """
            :type: None
            """
        @property
        def wasMuted(self) -> None:
            """
            :type: None
            """
        pass
    class LayersDidChange(Base, pxr.Tf.Notice, Boost.Python.instance):
        @staticmethod
        def GetLayers(*args, **kwargs) -> None: ...
        @staticmethod
        def GetSerialNumber(*args, **kwargs) -> None: ...
        pass
    class LayersDidChangeSentPerLayer(Base, pxr.Tf.Notice, Boost.Python.instance):
        @staticmethod
        def GetLayers(*args, **kwargs) -> None: ...
        @staticmethod
        def GetSerialNumber(*args, **kwargs) -> None: ...
        pass
    pass
class OpaqueValue(Boost.Python.instance):
    __instance_size__ = 32
    pass
class Path(Boost.Python.instance):
    class AncestorsRange(Boost.Python.instance):
        class _iterator(Boost.Python.instance):
            pass
        @staticmethod
        def GetPath(*args, **kwargs) -> None: ...
        __instance_size__ = 32
        pass
    class _IsValidPathStringResult(Boost.Python.instance):
        @property
        def errorMessage(self) -> None:
            """
            :type: None
            """
        __instance_size__ = 64
        pass
    @staticmethod
    def AppendChild(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendElementString(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendExpression(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendMapper(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendMapperArg(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendPath(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendProperty(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendRelationalAttribute(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendTarget(*args, **kwargs) -> None: ...
    @staticmethod
    def AppendVariantSelection(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsPrimVariantSelection(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsPropertyElements(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsTargetPath(*args, **kwargs) -> None: ...
    @staticmethod
    def FindLongestPrefix(*args, **kwargs) -> None: ...
    @staticmethod
    def FindLongestStrictPrefix(*args, **kwargs) -> None: ...
    @staticmethod
    def FindPrefixedRange(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAbsoluteRootOrPrimPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllTargetPathsRecursively(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAncestorsRange(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCommonPrefix(*args, **kwargs) -> None: ...
    @staticmethod
    def GetConciseRelativePaths(*args, **kwargs) -> None: ...
    @staticmethod
    def GetParentPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrefixes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrimOrPrimVariantSelectionPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrimPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVariantSelection(*args, **kwargs) -> None: ...
    @staticmethod
    def HasPrefix(*args, **kwargs) -> None: ...
    @staticmethod
    def IsAbsolutePath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsAbsoluteRootOrPrimPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsAbsoluteRootPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsExpressionPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMapperArgPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsMapperPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsNamespacedPropertyPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPrimPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPrimPropertyPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPrimVariantSelectionPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsPropertyPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsRelationalAttributePath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsRootPrimPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsTargetPath(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValidIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValidNamespacedIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValidPathString(*args, **kwargs) -> None: ...
    @staticmethod
    def JoinIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeAbsolutePath(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeRelativePath(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveAncestorPaths(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveCommonSuffix(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveDescendentPaths(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplaceName(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplacePrefix(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplaceTargetPath(*args, **kwargs) -> None: ...
    @staticmethod
    def StripAllVariantSelections(*args, **kwargs) -> None: ...
    @staticmethod
    def StripNamespace(*args, **kwargs) -> None: ...
    @staticmethod
    def StripPrefixNamespace(*args, **kwargs) -> None: ...
    @staticmethod
    def TokenizeIdentifier(*args, **kwargs) -> None: ...
    @property
    def elementString(self) -> None:
        """
        The string representation of the terminal component of this path.
        This path can be reconstructed via 
        thisPath.GetParentPath().AppendElementString(thisPath.element).
        None of absoluteRootPath, reflexiveRelativePath, nor emptyPath
        possess the above quality; their .elementString is the empty string.

        :type: None
        """
    @property
    def isEmpty(self) -> None:
        """
        :type: None
        """
    @property
    def name(self) -> None:
        """
        The name of the prim, property or relational
        attribute identified by the path.

        '' for EmptyPath.  '.' for ReflexiveRelativePath.
        '..' for a path ending in ParentPathElement.

        :type: None
        """
    @property
    def pathElementCount(self) -> None:
        """
        The number of path elements in this path.

        :type: None
        """
    @property
    def pathString(self) -> None:
        """
        The string representation of this path.

        :type: None
        """
    @property
    def targetPath(self) -> None:
        """
        The relational attribute target path for this path.

        EmptyPath if this is not a relational attribute path.

        :type: None
        """
    __instance_size__ = 32
    absoluteIndicator = '/'
    absoluteRootPath: pxr.Sdf.Path # value = Sdf.Path('/')
    childDelimiter = '/'
    emptyPath: pxr.Sdf.Path # value = Sdf.Path.emptyPath
    expressionIndicator = 'expression'
    mapperArgDelimiter = '.'
    mapperIndicator = 'mapper'
    namespaceDelimiter = ':'
    parentPathElement = '..'
    propertyDelimiter = '.'
    reflexiveRelativePath: pxr.Sdf.Path # value = Sdf.Path('.')
    relationshipTargetEnd = ']'
    relationshipTargetStart = '['
    pass
class PathArray(Boost.Python.instance):
    """
    An array of type SdfPath.
    """
    _isVtArray = True
    pass
class PathExpression(Boost.Python.instance):
    class ExpressionReference(Boost.Python.instance):
        @staticmethod
        def Weaker(*args, **kwargs) -> None: ...
        @property
        def name(self) -> None:
            """
            :type: None
            """
        @property
        def path(self) -> None:
            """
            :type: None
            """
        __instance_size__ = 64
        pass
    class Op(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'PathExpression'
        allValues: tuple # value = (Sdf.PathExpression.Complement, Sdf.PathExpression.ImpliedUnion, Sdf.PathExpression.Union, Sdf.PathExpression.Intersection, Sdf.PathExpression.Difference, Sdf.PathExpression.ExpressionRef, Sdf.PathExpression.Pattern)
        pass
    class PathPattern(Boost.Python.instance):
        @staticmethod
        def AppendChild(*args, **kwargs) -> None: ...
        @staticmethod
        def AppendProperty(*args, **kwargs) -> None: ...
        @staticmethod
        def AppendStretchIfPossible(*args, **kwargs) -> None: ...
        @staticmethod
        def CanAppendChild(*args, **kwargs) -> None: ...
        @staticmethod
        def CanAppendProperty(*args, **kwargs) -> None: ...
        @staticmethod
        def EveryDescendant(*args, **kwargs) -> None: ...
        @staticmethod
        def Everything(*args, **kwargs) -> None: ...
        @staticmethod
        def GetPrefix(*args, **kwargs) -> None: ...
        @staticmethod
        def GetText(*args, **kwargs) -> None: ...
        @staticmethod
        def HasLeadingStretch(*args, **kwargs) -> None: ...
        @staticmethod
        def HasTrailingStretch(*args, **kwargs) -> None: ...
        @staticmethod
        def IsProperty(*args, **kwargs) -> None: ...
        @staticmethod
        def Nothing(*args, **kwargs) -> None: ...
        @staticmethod
        def RemoveTrailingComponent(*args, **kwargs) -> None: ...
        @staticmethod
        def RemoveTrailingStretch(*args, **kwargs) -> None: ...
        @staticmethod
        def SetPrefix(*args, **kwargs) -> None: ...
        __instance_size__ = 88
        pass
    @staticmethod
    def ComposeOver(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsExpressionReferences(*args, **kwargs) -> None: ...
    @staticmethod
    def ContainsWeakerExpressionReference(*args, **kwargs) -> None: ...
    @staticmethod
    def Everything(*args, **kwargs) -> None: ...
    @staticmethod
    def GetText(*args, **kwargs) -> None: ...
    @staticmethod
    def IsAbsolute(*args, **kwargs) -> None: ...
    @staticmethod
    def IsComplete(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeAbsolute(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeAtom(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeComplement(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeOp(*args, **kwargs) -> None: ...
    @staticmethod
    def Nothing(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplacePrefix(*args, **kwargs) -> None: ...
    @staticmethod
    def ResolveReferences(*args, **kwargs) -> None: ...
    @staticmethod
    def Walk(*args, **kwargs) -> None: ...
    @staticmethod
    def WeakerRef(*args, **kwargs) -> None: ...
    Complement: pxr.Sdf.Op # value = Sdf.PathExpression.Complement
    Difference: pxr.Sdf.Op # value = Sdf.PathExpression.Difference
    ExpressionRef: pxr.Sdf.Op # value = Sdf.PathExpression.ExpressionRef
    ImpliedUnion: pxr.Sdf.Op # value = Sdf.PathExpression.ImpliedUnion
    Intersection: pxr.Sdf.Op # value = Sdf.PathExpression.Intersection
    Pattern: pxr.Sdf.Op # value = Sdf.PathExpression.Pattern
    Union: pxr.Sdf.Op # value = Sdf.PathExpression.Union
    __instance_size__ = 128
    pass
class PathExpressionArray(Boost.Python.instance):
    """
    An array of type SdfPathExpression.
    """
    _isVtArray = True
    pass
class PathListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class PathPattern():
    pass
class Payload(Boost.Python.instance):
    @property
    def assetPath(self) -> None:
        """
        :type: None
        """
    @property
    def layerOffset(self) -> None:
        """
        :type: None
        """
    @property
    def primPath(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 80
    pass
class PayloadListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class Permission(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.PermissionPublic, Sdf.PermissionPrivate)
    pass
class PredicateExpression(Boost.Python.instance):
    class FnArg(Boost.Python.instance):
        @staticmethod
        def Keyword(*args, **kwargs) -> None: ...
        @staticmethod
        def Positional(*args, **kwargs) -> None: ...
        @property
        def argName(self) -> None:
            """
            :type: None
            """
        @property
        def value(self) -> None:
            """
            :type: None
            """
        __instance_size__ = 72
        pass
    class FnCall(Boost.Python.instance):
        class Kind(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
            @staticmethod
            def GetValueFromName(*args, **kwargs) -> None: ...
            _baseName = 'PredicateExpression.FnCall'
            allValues: tuple # value = (Sdf.PredicateExpression.FnCall.BareCall, Sdf.PredicateExpression.FnCall.ColonCall, Sdf.PredicateExpression.FnCall.ParenCall)
            pass
        @property
        def args(self) -> None:
            """
            :type: None
            """
        @property
        def funcName(self) -> None:
            """
            :type: None
            """
        @property
        def kind(self) -> None:
            """
            :type: None
            """
        BareCall: pxr.Sdf.Kind # value = Sdf.PredicateExpression.FnCall.BareCall
        ColonCall: pxr.Sdf.Kind # value = Sdf.PredicateExpression.FnCall.ColonCall
        ParenCall: pxr.Sdf.Kind # value = Sdf.PredicateExpression.FnCall.ParenCall
        __instance_size__ = 88
        pass
    class Op(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'PredicateExpression'
        allValues: tuple # value = (Sdf.PredicateExpression.Call, Sdf.PredicateExpression.Not, Sdf.PredicateExpression.ImpliedAnd, Sdf.PredicateExpression.And, Sdf.PredicateExpression.Or)
        pass
    class _PredicateExpressionFnArgVector(Boost.Python.instance):
        @staticmethod
        def append(*args, **kwargs) -> None: ...
        @staticmethod
        def extend(*args, **kwargs) -> None: ...
        __instance_size__ = 48
        pass
    @staticmethod
    def GetParseError(*args, **kwargs) -> None: ...
    @staticmethod
    def GetText(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeCall(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeNot(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeOp(*args, **kwargs) -> None: ...
    @staticmethod
    def Walk(*args, **kwargs) -> None: ...
    And: pxr.Sdf.Op # value = Sdf.PredicateExpression.And
    Call: pxr.Sdf.Op # value = Sdf.PredicateExpression.Call
    ImpliedAnd: pxr.Sdf.Op # value = Sdf.PredicateExpression.ImpliedAnd
    Not: pxr.Sdf.Op # value = Sdf.PredicateExpression.Not
    Or: pxr.Sdf.Op # value = Sdf.PredicateExpression.Or
    __instance_size__ = 104
    pass
class PredicateFunctionResult(Boost.Python.instance):
    class Constancy(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'PredicateFunctionResult'
        allValues: tuple # value = (Sdf.PredicateFunctionResult.ConstantOverDescendants, Sdf.PredicateFunctionResult.MayVaryOverDescendants)
        pass
    @staticmethod
    def GetConstancy(*args, **kwargs) -> None: ...
    @staticmethod
    def GetValue(*args, **kwargs) -> None: ...
    @staticmethod
    def IsConstant(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeConstant(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeVarying(*args, **kwargs) -> None: ...
    @staticmethod
    def SetAndPropagateConstancy(*args, **kwargs) -> None: ...
    ConstantOverDescendants: pxr.Sdf.Constancy # value = Sdf.PredicateFunctionResult.ConstantOverDescendants
    MayVaryOverDescendants: pxr.Sdf.Constancy # value = Sdf.PredicateFunctionResult.MayVaryOverDescendants
    __instance_size__ = 32
    pass
class PrimSpec(Spec, Boost.Python.instance):
    @staticmethod
    def ApplyNameChildrenOrder(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyPropertyOrder(*args, **kwargs) -> None: ...
    @staticmethod
    def BlockVariantSelection(*args, **kwargs) -> None: ...
    @staticmethod
    def CanSetName(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearActive(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearInstanceable(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearKind(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearPayloadList(*args, **kwargs) -> None: 
        """
        Clears the payloads for this prim.
        """
    @staticmethod
    def ClearReferenceList(*args, **kwargs) -> None: 
        """
        Clears the references for this prim.
        """
    @staticmethod
    def GetAttributeAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetObjectAtPath(path) -> object: 
        """
        path: Path

        Returns a prim or property given its namespace path.

        If path is relative then it will be interpreted as relative to this prim.  If it is absolute then it will be interpreted as absolute in this prim's layer. The return type can be either PrimSpecPtr or PropertySpecPtr.
        """
    @staticmethod
    def GetPrimAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPropertyAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRelationshipAtPath(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVariantNames(*args, **kwargs) -> None: ...
    @staticmethod
    def HasActive(*args, **kwargs) -> None: ...
    @staticmethod
    def HasInstanceable(*args, **kwargs) -> None: ...
    @staticmethod
    def HasKind(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveProperty(*args, **kwargs) -> None: ...
    @property
    def active(self) -> None:
        """
        Whether this prim spec is active.
        The default value is true.

        :type: None
        """
    @property
    def assetInfo(self) -> None:
        """
        Returns the asset info dictionary for this prim.

        The default value is an empty dictionary.

        The asset info dictionary is used to annotate prims representing the root-prims of assets (generally organized as models) with various data related to asset management. For example, asset name, root layer identifier, asset version etc.

        :type: None
        """
    @property
    def attributes(self) -> None:
        """
        The attributes of this prim, as an ordered dictionary.

        :type: None
        """
    @property
    def comment(self) -> None:
        """
        The prim's comment string.

        :type: None
        """
    @property
    def customData(self) -> None:
        """
        The custom data for this prim.

        The default value for custom data is an empty dictionary.

        Custom data is for use by plugins or other non-tools supplied 
        extensions that need to be able to store data attached to arbitrary
        scene objects.  Note that if the only objects you want to store data
        on are prims, using custom attributes is probably a better choice.
        But if you need to possibly store this data on attributes or 
        relationships or as annotations on reference arcs, then custom data
        is an appropriate choice.

        :type: None
        """
    @property
    def documentation(self) -> None:
        """
        The prim's documentation string.

        :type: None
        """
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    @property
    def hasInheritPaths(self) -> None:
        """
        Returns true if this prim has inherits set.

        :type: None
        """
    @property
    def hasPayloads(self) -> None:
        """
        Returns true if this prim has payloads set.

        :type: None
        """
    @property
    def hasReferences(self) -> None:
        """
        Returns true if this prim has references set.

        :type: None
        """
    @property
    def hasSpecializes(self) -> None:
        """
        Returns true if this prim has specializes set.

        :type: None
        """
    @property
    def hidden(self) -> None:
        """
        Whether this prim spec will be hidden in browsers.
        The default value is false.

        :type: None
        """
    @property
    def inheritPathList(self) -> None:
        """
        A PathListEditor for the prim's inherit paths.

        The list of the inherit paths for this prim may be modified with this PathListEditor.

        A PathListEditor may express a list either as an explicit value or as a set of list editing operations.  See PathListEditor for more information.

        :type: None
        """
    @property
    def instanceable(self) -> None:
        """
        Whether this prim spec is flagged as instanceable.
        The default value is false.

        :type: None
        """
    @property
    def kind(self) -> None:
        """
        What kind of model this prim spec represents, if any.
        The default is an empty string

        :type: None
        """
    @property
    def name(self) -> None:
        """
        The prim's name.

        :type: None
        """
    @property
    def nameChildren(self) -> None:
        """
        The prim name children of this prim, as an ordered dictionary.

        Note that although this property is described as being read-only, you can modify the contents to add, remove, or reorder children.

        :type: None
        """
    @property
    def nameChildrenOrder(self) -> None:
        """
        Get/set the list of child names for this prim's 'reorder nameChildren' statement.

        :type: None
        """
    @property
    def nameParent(self) -> None:
        """
        The name parent of this prim.

        :type: None
        """
    @property
    def nameRoot(self) -> None:
        """
        The name pseudo-root of this prim.

        :type: None
        """
    @property
    def payloadList(self) -> None:
        """
        A PayloadListEditor for the prim's payloads.

        The list of the payloads for this prim may be modified with this PayloadListEditor.

        A PayloadListEditor may express a list either as an explicit value or as a set of list editing operations.  See PayloadListEditor for more information.

        :type: None
        """
    @property
    def permission(self) -> None:
        """
        The prim's permission restriction.
        The default value is SdfPermissionPublic.

        :type: None
        """
    @property
    def prefix(self) -> None:
        """
        The prim's prefix.

        :type: None
        """
    @property
    def prefixSubstitutions(self) -> None:
        """
        Dictionary of prefix substitutions.

        :type: None
        """
    @property
    def properties(self) -> None:
        """
        The properties of this prim, as an ordered dictionary.

        Note that although this property is described as being read-only, you can modify the contents to add, remove, or reorder properties.

        :type: None
        """
    @property
    def propertyOrder(self) -> None:
        """
        Get/set the list of property names for this prim's 'reorder properties' statement.

        :type: None
        """
    @property
    def realNameParent(self) -> None:
        """
        The name parent of this prim.

        :type: None
        """
    @property
    def referenceList(self) -> None:
        """
        A ReferenceListEditor for the prim's references.

        The list of the references for this prim may be modified with this ReferenceListEditor.

        A ReferenceListEditor may express a list either as an explicit value or as a set of list editing operations.  See ReferenceListEditor for more information.

        :type: None
        """
    @property
    def relationships(self) -> None:
        """
        The relationships of this prim, as an ordered dictionary.

        :type: None
        """
    @property
    def relocates(self) -> None:
        """
        An editing proxy for the prim's map of relocation paths.

        The map of source-to-target paths specifying namespace relocation may be set or cleared whole, or individual map entries may be added, removed, or edited.

        :type: None
        """
    @property
    def specializesList(self) -> None:
        """
        A PathListEditor for the prim's specializes.

        The list of the specializes for this prim may be modified with this PathListEditor.

        A PathListEditor may express a list either as an explicit value or as a set of list editing operations.  See PathListEditor for more information.

        :type: None
        """
    @property
    def specifier(self) -> None:
        """
        The prim's specifier (SpecifierDef or SpecifierOver).
        The default value is SpecifierOver.

        :type: None
        """
    @property
    def suffix(self) -> None:
        """
        The prim's suffix.

        :type: None
        """
    @property
    def suffixSubstitutions(self) -> None:
        """
        Dictionary of prefix substitutions.

        :type: None
        """
    @property
    def symmetricPeer(self) -> None:
        """
        The prims's symmetric peer.

        :type: None
        """
    @property
    def symmetryArguments(self) -> None:
        """
        Dictionary with prim symmetry arguments.

        Although this property is marked read-only, you can modify the contents to add, change, and clear symmetry arguments.

        :type: None
        """
    @property
    def symmetryFunction(self) -> None:
        """
        The prim's symmetry function.

        :type: None
        """
    @property
    def typeName(self) -> None:
        """
        The type of this prim.

        :type: None
        """
    @property
    def variantSelections(self) -> None:
        """
        Dictionary whose keys are variant set names and whose values are the variants chosen for each set.

        Although this property is marked read-only, you can modify the contents to add, change, and clear variants.

        :type: None
        """
    @property
    def variantSetNameList(self) -> None:
        """
        A StringListEditor for the names of the variant 
        sets for this prim.

        The list of the names of the variants sets of this prim may be
        modified with this StringListEditor.

        A StringListEditor may express a list either as an explicit value or as a set of list editing operations.  See StringListEditor for more information.

        Although this property is marked as read-only, the returned object is modifiable.

        :type: None
        """
    @property
    def variantSets(self) -> None:
        """
        The VariantSetSpecs for this prim indexed by name.

        Although this property is marked as read-only, you can 
        modify the contents to remove variant sets.  New variant sets 
        are created by creating them with the prim as the owner.

        Although this property is marked as read-only, the returned object
        is modifiable.

        :type: None
        """
    ActiveKey = 'active'
    AnyTypeToken = '__AnyType__'
    CommentKey = 'comment'
    CustomDataKey = 'customData'
    DisplayName = 'displayName'
    DocumentationKey = 'documentation'
    HiddenKey = 'hidden'
    InheritPathsKey = 'inheritPaths'
    KindKey = 'kind'
    PayloadKey = 'payload'
    PermissionKey = 'permission'
    PrefixKey = 'prefix'
    PrefixSubstitutionsKey = 'prefixSubstitutions'
    PrimOrderKey = 'primOrder'
    PropertyOrderKey = 'propertyOrder'
    ReferencesKey = 'references'
    RelocatesKey = 'relocates'
    SpecializesKey = 'specializes'
    SpecifierKey = 'specifier'
    SymmetricPeerKey = 'symmetricPeer'
    SymmetryArgumentsKey = 'symmetryArguments'
    SymmetryFunctionKey = 'symmetryFunction'
    TypeNameKey = 'typeName'
    VariantSelectionKey = 'variantSelection'
    VariantSetNamesKey = 'variantSetNames'
    pass
class PropertySpec(Spec, Boost.Python.instance):
    @staticmethod
    def ClearDefaultValue(*args, **kwargs) -> None: ...
    @staticmethod
    def HasDefaultValue(*args, **kwargs) -> None: ...
    @property
    def assetInfo(self) -> None:
        """
        Returns the asset info dictionary for this property.

        The default value is an empty dictionary.

        The asset info dictionary is used to annotate SdfAssetPath-valued attributes pointing to the root-prims of assets (generally organized as models) with various data related to asset management. For example, asset name, root layer identifier, asset version etc.

        Note: It is only valid to author assetInfo on attributes that are of type SdfAssetPath.

        :type: None
        """
    @property
    def comment(self) -> None:
        """
        A comment describing the property.

        :type: None
        """
    @property
    def custom(self) -> None:
        """
        Whether this property spec declares a custom attribute.

        :type: None
        """
    @property
    def customData(self) -> None:
        """
        The property's custom data.

        The default value for custom data is an empty dictionary.

        Custom data is for use by plugins or other non-tools supplied 
        extensions that need to be able to store data attached to arbitrary
        scene objects.  Note that if the only objects you want to store data
        on are prims, using custom attributes is probably a better choice.
        But if you need to possibly store this data on attributes or 
        relationships or as annotations on reference arcs, then custom data
        is an appropriate choice.

        :type: None
        """
    @property
    def default(self) -> None:
        """
        The default value of this property.

        :type: None
        """
    @property
    def displayGroup(self) -> None:
        """
        DisplayGroup for the property.

        :type: None
        """
    @property
    def displayName(self) -> None:
        """
        DisplayName for the property.

        :type: None
        """
    @property
    def documentation(self) -> None:
        """
        Documentation for the property.

        :type: None
        """
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    @property
    def hasOnlyRequiredFields(self) -> None:
        """
        Indicates whether this spec has any significant data other 
        than just what is necessary for instantiation.

        This is a less strict version of isInert, returning True if 
        the spec contains as much as the type and name.

        :type: None
        """
    @property
    def hidden(self) -> None:
        """
        Whether this property will be hidden in browsers.

        :type: None
        """
    @property
    def name(self) -> None:
        """
        The name of the property.

        :type: None
        """
    @property
    def owner(self) -> None:
        """
        The owner of this property.  Either a relationship or a prim.

        :type: None
        """
    @property
    def permission(self) -> None:
        """
        The property's permission restriction.

        :type: None
        """
    @property
    def prefix(self) -> None:
        """
        Prefix for the property.

        :type: None
        """
    @property
    def symmetricPeer(self) -> None:
        """
        The property's symmetric peer.

        :type: None
        """
    @property
    def symmetryArguments(self) -> None:
        """
        Dictionary with property symmetry arguments.

        Although this property is marked read-only, you can modify the contents to add, change, and clear symmetry arguments.

        :type: None
        """
    @property
    def symmetryFunction(self) -> None:
        """
        The property's symmetry function.

        :type: None
        """
    @property
    def variability(self) -> None:
        """
        Returns the variability of the property.

        An attribute's variability may be Varying
        Uniform, Config or Computed.
        For an attribute, the default is Varying, for a relationship the default is Uniform.

        Varying relationships may be directly authored 'animating' targetpaths over time.
        Varying attributes may be directly authored, animated and 
        affected on by Actions.  They are the most flexible.

        Uniform attributes may be authored only with non-animated values
        (default values).  They cannot be affected by Actions, but they
        can be connected to other Uniform attributes.

        Config attributes are the same as Uniform except that a Prim
        can choose to alter its collection of built-in properties based
        on the values of its Config attributes.

        Computed attributes may not be authored in scene description.
        Prims determine the values of their Computed attributes through
        Prim-specific computation.  They may not be connected.

        :type: None
        """
    AssetInfoKey = 'assetInfo'
    CommentKey = 'comment'
    CustomDataKey = 'customData'
    CustomKey = 'custom'
    DisplayGroupKey = 'displayGroup'
    DisplayNameKey = 'displayName'
    DocumentationKey = 'documentation'
    HiddenKey = 'hidden'
    PermissionKey = 'permission'
    PrefixKey = 'prefix'
    SymmetricPeerKey = 'symmetricPeer'
    SymmetryArgumentsKey = 'symmetryArguments'
    SymmetryFunctionKey = 'symmetryFunction'
    pass
class PseudoRootSpec(PrimSpec, Spec, Boost.Python.instance):
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    pass
class Reference(Boost.Python.instance):
    @staticmethod
    def IsInternal(*args, **kwargs) -> None: ...
    @property
    def assetPath(self) -> None:
        """
        :type: None
        """
    @property
    def customData(self) -> None:
        """
        :type: None
        """
    @property
    def layerOffset(self) -> None:
        """
        :type: None
        """
    @property
    def primPath(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 88
    pass
class ReferenceListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class RelationshipSpec(PropertySpec, Spec, Boost.Python.instance):
    @staticmethod
    def RemoveTargetPath(*args, **kwargs) -> None: ...
    @staticmethod
    def ReplaceTargetPath(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    @property
    def noLoadHint(self) -> None:
        """
        whether the target must be loaded to load the prim this
        relationship is attached to.

        :type: None
        """
    @property
    def targetPathList(self) -> None:
        """
        A PathListEditor for the relationship's target paths.

        The list of the target paths for this relationship may be
        modified with this PathListEditor.

        A PathListEditor may express a list either as an explicit 
        value or as a set of list editing operations.  See PathListEditor 
        for more information.

        :type: None
        """
    TargetsKey = 'targetPaths'
    pass
class Spec(Boost.Python.instance):
    @staticmethod
    def ClearInfo(*args, **kwargs) -> None: 
        """
        key : string
        nClears the value for scene spec info with the given key. After calling this, HasInfo() will return false. To make HasInfo() return true, set a value for that scene spec info.
        """
    @staticmethod
    def GetAsText(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFallbackForInfo(*args, **kwargs) -> None: 
        """
        key : string

        Returns the fallback value for the given key. 
        """
    @staticmethod
    def GetInfo(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMetaDataDisplayGroup(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMetaDataInfoKeys(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTypeForInfo(*args, **kwargs) -> None: 
        """
        key : string

        Returns the type of value for the given key. 
        """
    @staticmethod
    def HasInfo(key) -> bool: 
        """
        key : string

        Returns whether there is a setting for the scene spec info with the given key.

        When asked for a value for one of its scene spec info, a valid value will always be returned. But if this API returns false for a scene spec info, the value of that info will be the defined default value. 

        (XXX: This may change such that it is an error to ask for a value when there is none).

        When dealing with a composedLayer, it is not necessary to worry about whether a scene spec info 'has a value' because the composed layer will always have a valid value, even if it is the default.

        A spec may or may not have an expressed value for some of its scene spec info.
        """
    @staticmethod
    def IsInert(*args, **kwargs) -> None: 
        """
        Indicates whether this spec has any significant data. If ignoreChildren is true, child scenegraph objects will be ignored.
        """
    @staticmethod
    def ListInfoKeys(*args, **kwargs) -> None: ...
    @staticmethod
    def SetInfo(*args, **kwargs) -> None: ...
    @staticmethod
    def SetInfoDictionaryValue(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    @property
    def isInert(self) -> None:
        """
        Indicates whether this spec has any significant data. This is for backwards compatibility, use IsInert instead.

        Compatibility note: prior to presto 1.9, isInert (then isEmpty) was true for otherwise inert PrimSpecs with inert inherits, references, or variant sets. isInert is now false in such conditions.

        :type: None
        """
    @property
    def layer(self) -> None:
        """
        The owning layer.

        :type: None
        """
    @property
    def path(self) -> None:
        """
        The absolute scene path.

        :type: None
        """
    pass
class SpecType(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.SpecTypeUnknown, Sdf.SpecTypeAttribute, Sdf.SpecTypeConnection, Sdf.SpecTypeExpression, Sdf.SpecTypeMapper, Sdf.SpecTypeMapperArg, Sdf.SpecTypePrim, Sdf.SpecTypePseudoRoot, Sdf.SpecTypeRelationship, Sdf.SpecTypeRelationshipTarget, Sdf.SpecTypeVariant, Sdf.SpecTypeVariantSet)
    pass
class Specifier(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.SpecifierDef, Sdf.SpecifierOver, Sdf.SpecifierClass)
    pass
class StringListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class TimeCode(Boost.Python.instance):
    @staticmethod
    def GetValue(*args, **kwargs) -> None: ...
    __instance_size__ = 32
    pass
class TimeCodeArray(Boost.Python.instance):
    """
    An array of type SdfTimeCode.
    """
    _isVtArray = True
    pass
class TokenListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class UInt64ListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class UIntListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class UnregisteredValue(Boost.Python.instance):
    @property
    def value(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class UnregisteredValueListOp(Boost.Python.instance):
    @staticmethod
    def ApplyOperations(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearAndMakeExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExplicit(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAddedOrExplicitItems(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAppliedItems(*args, **kwargs) -> None: ...
    @staticmethod
    def HasItem(*args, **kwargs) -> None: ...
    @property
    def addedItems(self) -> None:
        """
        :type: None
        """
    @property
    def appendedItems(self) -> None:
        """
        :type: None
        """
    @property
    def deletedItems(self) -> None:
        """
        :type: None
        """
    @property
    def explicitItems(self) -> None:
        """
        :type: None
        """
    @property
    def isExplicit(self) -> None:
        """
        :type: None
        """
    @property
    def orderedItems(self) -> None:
        """
        :type: None
        """
    @property
    def prependedItems(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 176
    pass
class ValueBlock(Boost.Python.instance):
    __instance_size__ = 32
    pass
class ValueRoleNames(Boost.Python.instance):
    Color = 'Color'
    EdgeIndex = 'EdgeIndex'
    FaceIndex = 'FaceIndex'
    Frame = 'Frame'
    Group = 'Group'
    Normal = 'Normal'
    Point = 'Point'
    PointIndex = 'PointIndex'
    TextureCoordinate = 'TextureCoordinate'
    Transform = 'Transform'
    Vector = 'Vector'
    pass
class ValueTypeName(Boost.Python.instance):
    @property
    def aliasesAsStrings(self) -> None:
        """
        :type: None
        """
    @property
    def arrayType(self) -> None:
        """
        :type: None
        """
    @property
    def cppTypeName(self) -> None:
        """
        :type: None
        """
    @property
    def defaultUnit(self) -> None:
        """
        :type: None
        """
    @property
    def defaultValue(self) -> None:
        """
        :type: None
        """
    @property
    def isArray(self) -> None:
        """
        :type: None
        """
    @property
    def isScalar(self) -> None:
        """
        :type: None
        """
    @property
    def role(self) -> None:
        """
        :type: None
        """
    @property
    def scalarType(self) -> None:
        """
        :type: None
        """
    @property
    def type(self) -> None:
        """
        :type: None
        """
    pass
class ValueTypeNames(Boost.Python.instance):
    @staticmethod
    def Find(*args, **kwargs) -> None: ...
    Asset: pxr.Sdf.ValueTypeName
    AssetArray: pxr.Sdf.ValueTypeName
    Bool: pxr.Sdf.ValueTypeName
    BoolArray: pxr.Sdf.ValueTypeName
    Color3d: pxr.Sdf.ValueTypeName
    Color3dArray: pxr.Sdf.ValueTypeName
    Color3f: pxr.Sdf.ValueTypeName
    Color3fArray: pxr.Sdf.ValueTypeName
    Color3h: pxr.Sdf.ValueTypeName
    Color3hArray: pxr.Sdf.ValueTypeName
    Color4d: pxr.Sdf.ValueTypeName
    Color4dArray: pxr.Sdf.ValueTypeName
    Color4f: pxr.Sdf.ValueTypeName
    Color4fArray: pxr.Sdf.ValueTypeName
    Color4h: pxr.Sdf.ValueTypeName
    Color4hArray: pxr.Sdf.ValueTypeName
    Double: pxr.Sdf.ValueTypeName
    Double2: pxr.Sdf.ValueTypeName
    Double2Array: pxr.Sdf.ValueTypeName
    Double3: pxr.Sdf.ValueTypeName
    Double3Array: pxr.Sdf.ValueTypeName
    Double4: pxr.Sdf.ValueTypeName
    Double4Array: pxr.Sdf.ValueTypeName
    DoubleArray: pxr.Sdf.ValueTypeName
    Float: pxr.Sdf.ValueTypeName
    Float2: pxr.Sdf.ValueTypeName
    Float2Array: pxr.Sdf.ValueTypeName
    Float3: pxr.Sdf.ValueTypeName
    Float3Array: pxr.Sdf.ValueTypeName
    Float4: pxr.Sdf.ValueTypeName
    Float4Array: pxr.Sdf.ValueTypeName
    FloatArray: pxr.Sdf.ValueTypeName
    Frame4d: pxr.Sdf.ValueTypeName
    Frame4dArray: pxr.Sdf.ValueTypeName
    Group: pxr.Sdf.ValueTypeName
    Half: pxr.Sdf.ValueTypeName
    Half2: pxr.Sdf.ValueTypeName
    Half2Array: pxr.Sdf.ValueTypeName
    Half3: pxr.Sdf.ValueTypeName
    Half3Array: pxr.Sdf.ValueTypeName
    Half4: pxr.Sdf.ValueTypeName
    Half4Array: pxr.Sdf.ValueTypeName
    HalfArray: pxr.Sdf.ValueTypeName
    Int: pxr.Sdf.ValueTypeName
    Int2: pxr.Sdf.ValueTypeName
    Int2Array: pxr.Sdf.ValueTypeName
    Int3: pxr.Sdf.ValueTypeName
    Int3Array: pxr.Sdf.ValueTypeName
    Int4: pxr.Sdf.ValueTypeName
    Int4Array: pxr.Sdf.ValueTypeName
    Int64: pxr.Sdf.ValueTypeName
    Int64Array: pxr.Sdf.ValueTypeName
    IntArray: pxr.Sdf.ValueTypeName
    Matrix2d: pxr.Sdf.ValueTypeName
    Matrix2dArray: pxr.Sdf.ValueTypeName
    Matrix3d: pxr.Sdf.ValueTypeName
    Matrix3dArray: pxr.Sdf.ValueTypeName
    Matrix4d: pxr.Sdf.ValueTypeName
    Matrix4dArray: pxr.Sdf.ValueTypeName
    Normal3d: pxr.Sdf.ValueTypeName
    Normal3dArray: pxr.Sdf.ValueTypeName
    Normal3f: pxr.Sdf.ValueTypeName
    Normal3fArray: pxr.Sdf.ValueTypeName
    Normal3h: pxr.Sdf.ValueTypeName
    Normal3hArray: pxr.Sdf.ValueTypeName
    Opaque: pxr.Sdf.ValueTypeName
    PathExpression: pxr.Sdf.ValueTypeName
    PathExpressionArray: pxr.Sdf.ValueTypeName
    Point3d: pxr.Sdf.ValueTypeName
    Point3dArray: pxr.Sdf.ValueTypeName
    Point3f: pxr.Sdf.ValueTypeName
    Point3fArray: pxr.Sdf.ValueTypeName
    Point3h: pxr.Sdf.ValueTypeName
    Point3hArray: pxr.Sdf.ValueTypeName
    Quatd: pxr.Sdf.ValueTypeName
    QuatdArray: pxr.Sdf.ValueTypeName
    Quatf: pxr.Sdf.ValueTypeName
    QuatfArray: pxr.Sdf.ValueTypeName
    Quath: pxr.Sdf.ValueTypeName
    QuathArray: pxr.Sdf.ValueTypeName
    String: pxr.Sdf.ValueTypeName
    StringArray: pxr.Sdf.ValueTypeName
    TexCoord2d: pxr.Sdf.ValueTypeName
    TexCoord2dArray: pxr.Sdf.ValueTypeName
    TexCoord2f: pxr.Sdf.ValueTypeName
    TexCoord2fArray: pxr.Sdf.ValueTypeName
    TexCoord2h: pxr.Sdf.ValueTypeName
    TexCoord2hArray: pxr.Sdf.ValueTypeName
    TexCoord3d: pxr.Sdf.ValueTypeName
    TexCoord3dArray: pxr.Sdf.ValueTypeName
    TexCoord3f: pxr.Sdf.ValueTypeName
    TexCoord3fArray: pxr.Sdf.ValueTypeName
    TexCoord3h: pxr.Sdf.ValueTypeName
    TexCoord3hArray: pxr.Sdf.ValueTypeName
    TimeCode: pxr.Sdf.ValueTypeName
    TimeCodeArray: pxr.Sdf.ValueTypeName
    Token: pxr.Sdf.ValueTypeName
    TokenArray: pxr.Sdf.ValueTypeName
    UChar: pxr.Sdf.ValueTypeName
    UCharArray: pxr.Sdf.ValueTypeName
    UInt: pxr.Sdf.ValueTypeName
    UInt64: pxr.Sdf.ValueTypeName
    UInt64Array: pxr.Sdf.ValueTypeName
    UIntArray: pxr.Sdf.ValueTypeName
    Vector3d: pxr.Sdf.ValueTypeName
    Vector3dArray: pxr.Sdf.ValueTypeName
    Vector3f: pxr.Sdf.ValueTypeName
    Vector3fArray: pxr.Sdf.ValueTypeName
    Vector3h: pxr.Sdf.ValueTypeName
    Vector3hArray: pxr.Sdf.ValueTypeName
    pass
class Variability(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Sdf.VariabilityVarying, Sdf.VariabilityUniform)
    pass
class VariableExpression(Boost.Python.instance):
    class Result(Boost.Python.instance):
        @property
        def errors(self) -> None:
            """
            :type: None
            """
        @property
        def usedVariables(self) -> None:
            """
            :type: None
            """
        @property
        def value(self) -> None:
            """
            :type: None
            """
        pass
    @staticmethod
    def Evaluate(*args, **kwargs) -> None: ...
    @staticmethod
    def GetErrors(*args, **kwargs) -> None: ...
    @staticmethod
    def IsExpression(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValidVariableType(*args, **kwargs) -> None: ...
    __instance_size__ = 96
    pass
class VariantSetSpec(Spec, Boost.Python.instance):
    @staticmethod
    def RemoveVariant(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    @property
    def name(self) -> None:
        """
        The variant set's name.

        :type: None
        """
    @property
    def owner(self) -> None:
        """
        The prim that this variant set belongs to.

        :type: None
        """
    @property
    def variantList(self) -> None:
        """
        The variants in this variant set as a list.

        :type: None
        """
    @property
    def variants(self) -> None:
        """
        The variants in this variant set as a dict.

        :type: None
        """
    pass
class VariantSpec(Spec, Boost.Python.instance):
    @staticmethod
    def GetVariantNames(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        :type: None
        """
    @property
    def name(self) -> None:
        """
        The variant's name.

        :type: None
        """
    @property
    def owner(self) -> None:
        """
        The variant set that this variant belongs to.

        :type: None
        """
    @property
    def primSpec(self) -> None:
        """
        The root prim of this variant.

        :type: None
        """
    @property
    def variantSets(self) -> None:
        """
        :type: None
        """
    pass
class _PathPatternCanAppendResult(Boost.Python.instance):
    @property
    def reason(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
def ComputeAssetPathRelativeToLayer(*args, **kwargs) -> None:
    pass
def ConvertToValidMetadataDictionary(*args, **kwargs) -> None:
    pass
def ConvertUnit(*args, **kwargs) -> None:
    """
    Convert a unit of measurement to a compatible unit.
    """
def CopySpec(*args, **kwargs) -> None:
    pass
def CreatePrimAttributeInLayer(*args, **kwargs) -> None:
    pass
def CreatePrimInLayer(*args, **kwargs) -> None:
    pass
def CreateRelationshipInLayer(*args, **kwargs) -> None:
    pass
def CreateVariantInLayer(*args, **kwargs) -> None:
    pass
def DefaultUnit(*args, **kwargs) -> None:
    """
    For a given unit of measurement get the default compatible unit.

    For a given typeName ('Vector', 'Point' etc.) get the default unit of measurement.
    """
def GetNameForUnit(*args, **kwargs) -> None:
    pass
def GetTypeForValueTypeName(*args, **kwargs) -> None:
    pass
def GetUnitFromName(*args, **kwargs) -> None:
    pass
def GetValueTypeNameForValue(*args, **kwargs) -> None:
    pass
def JustCreatePrimAttributeInLayer(*args, **kwargs) -> None:
    pass
def JustCreatePrimInLayer(*args, **kwargs) -> None:
    pass
def JustCreateRelationshipInLayer(*args, **kwargs) -> None:
    pass
def UnitCategory(*args, **kwargs) -> None:
    """
    For a given unit of measurement get the unit category.
    """
def ValueHasValidType(*args, **kwargs) -> None:
    pass
def _DumpPathStats(*args, **kwargs) -> None:
    pass
def _MakeBasicMatchEval(*args, **kwargs) -> None:
    pass
def _PathGetDebuggerPathText(*args, **kwargs) -> None:
    pass
def _PathStress(*args, **kwargs) -> None:
    pass
def _TestTakeOwnership(*args, **kwargs) -> None:
    pass
AngularUnitDegrees: pxr.Sdf.AngularUnit # value = Sdf.AngularUnitDegrees
AngularUnitRadians: pxr.Sdf.AngularUnit # value = Sdf.AngularUnitRadians
AuthoringErrorUnrecognizedFields: pxr.Sdf.AuthoringError # value = Sdf.AuthoringErrorUnrecognizedFields
AuthoringErrorUnrecognizedSpecType: pxr.Sdf.AuthoringError # value = Sdf.AuthoringErrorUnrecognizedSpecType
DimensionlessUnitDefault: pxr.Sdf.DimensionlessUnit # value = Sdf.DimensionlessUnitDefault
DimensionlessUnitPercent: pxr.Sdf.DimensionlessUnit # value = Sdf.DimensionlessUnitPercent
LengthUnitCentimeter: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitCentimeter
LengthUnitDecimeter: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitDecimeter
LengthUnitFoot: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitFoot
LengthUnitInch: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitInch
LengthUnitKilometer: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitKilometer
LengthUnitMeter: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitMeter
LengthUnitMile: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitMile
LengthUnitMillimeter: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitMillimeter
LengthUnitYard: pxr.Sdf.LengthUnit # value = Sdf.LengthUnitYard
ListOpTypeAdded: pxr.Sdf.ListOpType # value = Sdf.ListOpTypeAdded
ListOpTypeAppended: pxr.Sdf.ListOpType # value = Sdf.ListOpTypeAppended
ListOpTypeDeleted: pxr.Sdf.ListOpType # value = Sdf.ListOpTypeDeleted
ListOpTypeExplicit: pxr.Sdf.ListOpType # value = Sdf.ListOpTypeExplicit
ListOpTypeOrdered: pxr.Sdf.ListOpType # value = Sdf.ListOpTypeOrdered
ListOpTypePrepended: pxr.Sdf.ListOpType # value = Sdf.ListOpTypePrepended
PermissionPrivate: pxr.Sdf.Permission # value = Sdf.PermissionPrivate
PermissionPublic: pxr.Sdf.Permission # value = Sdf.PermissionPublic
SpecTypeAttribute: pxr.Sdf.SpecType # value = Sdf.SpecTypeAttribute
SpecTypeConnection: pxr.Sdf.SpecType # value = Sdf.SpecTypeConnection
SpecTypeExpression: pxr.Sdf.SpecType # value = Sdf.SpecTypeExpression
SpecTypeMapper: pxr.Sdf.SpecType # value = Sdf.SpecTypeMapper
SpecTypeMapperArg: pxr.Sdf.SpecType # value = Sdf.SpecTypeMapperArg
SpecTypePrim: pxr.Sdf.SpecType # value = Sdf.SpecTypePrim
SpecTypePseudoRoot: pxr.Sdf.SpecType # value = Sdf.SpecTypePseudoRoot
SpecTypeRelationship: pxr.Sdf.SpecType # value = Sdf.SpecTypeRelationship
SpecTypeRelationshipTarget: pxr.Sdf.SpecType # value = Sdf.SpecTypeRelationshipTarget
SpecTypeUnknown: pxr.Sdf.SpecType # value = Sdf.SpecTypeUnknown
SpecTypeVariant: pxr.Sdf.SpecType # value = Sdf.SpecTypeVariant
SpecTypeVariantSet: pxr.Sdf.SpecType # value = Sdf.SpecTypeVariantSet
SpecifierClass: pxr.Sdf.Specifier # value = Sdf.SpecifierClass
SpecifierDef: pxr.Sdf.Specifier # value = Sdf.SpecifierDef
SpecifierOver: pxr.Sdf.Specifier # value = Sdf.SpecifierOver
VariabilityUniform: pxr.Sdf.Variability # value = Sdf.VariabilityUniform
VariabilityVarying: pxr.Sdf.Variability # value = Sdf.VariabilityVarying
__MFB_FULL_PACKAGE_NAME = 'sdf'
