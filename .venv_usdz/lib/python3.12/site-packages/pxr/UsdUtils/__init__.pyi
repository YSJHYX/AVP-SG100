from __future__ import annotations
import pxr.UsdUtils._usdUtils
import typing
import Boost.Python

__all__ = [
    "AuthorCollection",
    "CoalescingDiagnosticDelegate",
    "CoalescingDiagnosticDelegateItem",
    "CoalescingDiagnosticDelegateSharedItem",
    "CoalescingDiagnosticDelegateUnsharedItem",
    "ComputeAllDependencies",
    "ComputeCollectionIncludesAndExcludes",
    "ComputeUsdStageStats",
    "ConditionalAbortDiagnosticDelegate",
    "ConditionalAbortDiagnosticDelegateErrorFilters",
    "CopyLayerMetadata",
    "CreateCollections",
    "CreateNewARKitUsdzPackage",
    "CreateNewUsdzPackage",
    "DependencyInfo",
    "ExtractExternalReferences",
    "ExtractExternalReferencesParams",
    "FlattenLayerStack",
    "FlattenLayerStackResolveAssetPath",
    "GenerateClipManifestName",
    "GenerateClipTopologyName",
    "GetAlphaAttributeNameForColor",
    "GetDirtyLayers",
    "GetMaterialsScopeName",
    "GetModelNameFromRootLayer",
    "GetPrefName",
    "GetPrimAtPathWithForwarding",
    "GetPrimaryCameraName",
    "GetPrimaryUVSetName",
    "GetRegisteredVariantSets",
    "LocalizeAsset",
    "ModifyAssetPaths",
    "RegisteredVariantSet",
    "SparseAttrValueWriter",
    "SparseValueWriter",
    "StageCache",
    "StitchClips",
    "StitchClipsManifest",
    "StitchClipsTemplate",
    "StitchClipsTopology",
    "StitchInfo",
    "StitchLayers",
    "TimeCodeRange",
    "UninstancePrimAtPath",
    "UsdStageStatsKeys"
]


class CoalescingDiagnosticDelegate(Boost.Python.instance):
    @staticmethod
    def DumpCoalescedDiagnosticsToStderr(*args, **kwargs) -> None: ...
    @staticmethod
    def DumpCoalescedDiagnosticsToStdout(*args, **kwargs) -> None: ...
    @staticmethod
    def DumpUncoalescedDiagnostics(*args, **kwargs) -> None: ...
    @staticmethod
    def TakeCoalescedDiagnostics(*args, **kwargs) -> None: ...
    @staticmethod
    def TakeUncoalescedDiagnostics(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class CoalescingDiagnosticDelegateItem(Boost.Python.instance):
    @property
    def sharedItem(self) -> None:
        """
        :type: None
        """
    @property
    def unsharedItems(self) -> None:
        """
        :type: None
        """
    pass
class CoalescingDiagnosticDelegateSharedItem(Boost.Python.instance):
    @property
    def sourceFileName(self) -> None:
        """
        :type: None
        """
    @property
    def sourceFunction(self) -> None:
        """
        :type: None
        """
    @property
    def sourceLineNumber(self) -> None:
        """
        :type: None
        """
    pass
class CoalescingDiagnosticDelegateUnsharedItem(Boost.Python.instance):
    @property
    def commentary(self) -> None:
        """
        :type: None
        """
    @property
    def context(self) -> None:
        """
        :type: None
        """
    pass
class ConditionalAbortDiagnosticDelegate(Boost.Python.instance):
    __instance_size__ = 128
    pass
class ConditionalAbortDiagnosticDelegateErrorFilters(Boost.Python.instance):
    @staticmethod
    def GetCodePathFilters(*args, **kwargs) -> None: ...
    @staticmethod
    def GetStringFilters(*args, **kwargs) -> None: ...
    @staticmethod
    def SetCodePathFilters(*args, **kwargs) -> None: ...
    @staticmethod
    def SetStringFilters(*args, **kwargs) -> None: ...
    __instance_size__ = 72
    pass
class DependencyInfo(Boost.Python.instance):
    @property
    def assetPath(self) -> None:
        """
        :type: None
        """
    @property
    def dependencies(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 80
    pass
class ExtractExternalReferencesParams(Boost.Python.instance):
    @staticmethod
    def GetResolveUdimPaths(*args, **kwargs) -> None: ...
    @staticmethod
    def SetResolveUdimPaths(*args, **kwargs) -> None: ...
    __instance_size__ = 32
    pass
class RegisteredVariantSet(Boost.Python.instance):
    """
    Info for registered variant set
    """
    class SelectionExportPolicy(Boost.Python.enum, int):
        Always = pxr.UsdUtils.SelectionExportPolicy.Always
        IfAuthored = pxr.UsdUtils.SelectionExportPolicy.IfAuthored
        Never = pxr.UsdUtils.SelectionExportPolicy.Never
        __slots__ = ()
        names = {'IfAuthored': pxr.UsdUtils.SelectionExportPolicy.IfAuthored, 'Always': pxr.UsdUtils.SelectionExportPolicy.Always, 'Never': pxr.UsdUtils.SelectionExportPolicy.Never}
        values = {1: pxr.UsdUtils.SelectionExportPolicy.IfAuthored, 2: pxr.UsdUtils.SelectionExportPolicy.Always, 0: pxr.UsdUtils.SelectionExportPolicy.Never}
        pass
    @property
    def name(self) -> None:
        """
        :type: None
        """
    @property
    def selectionExportPolicy(self) -> None:
        """
        :type: None
        """
    pass
class SparseAttrValueWriter(Boost.Python.instance):
    @staticmethod
    def SetTimeSample(*args, **kwargs) -> None: ...
    pass
class SparseValueWriter(Boost.Python.instance):
    @staticmethod
    def GetSparseAttrValueWriters(*args, **kwargs) -> None: ...
    @staticmethod
    def SetAttribute(*args, **kwargs) -> None: ...
    __instance_size__ = 80
    pass
class StageCache(Boost.Python.instance):
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSessionLayerForVariantSelections(*args, **kwargs) -> None: ...
    __instance_size__ = 32
    pass
class TimeCodeRange(Boost.Python.instance):
    class Tokens(Boost.Python.instance):
        EmptyTimeCodeRange = 'NONE'
        RangeSeparator = ':'
        StrideSeparator = 'x'
        pass
    class _Iterator(Boost.Python.instance):
        pass
    @staticmethod
    def CreateFromFrameSpec(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValid(*args, **kwargs) -> None: ...
    @staticmethod
    def empty(*args, **kwargs) -> None: ...
    @property
    def endTimeCode(self) -> None:
        """
        :type: None
        """
    @property
    def frameSpec(self) -> None:
        """
        :type: None
        """
    @property
    def startTimeCode(self) -> None:
        """
        :type: None
        """
    @property
    def stride(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
class UsdStageStatsKeys(Boost.Python.instance):
    activePrimCount = 'activePrimCount'
    approxMemoryInMb = 'approxMemoryInMb'
    assetCount = 'assetCount'
    inactivePrimCount = 'inactivePrimCount'
    instanceCount = 'instanceCount'
    instancedModelCount = 'instancedModelCount'
    modelCount = 'modelCount'
    primCounts = 'primCounts'
    primCountsByType = 'primCountsByType'
    primary = 'primary'
    prototypeCount = 'prototypeCount'
    prototypes = 'prototypes'
    pureOverCount = 'pureOverCount'
    totalInstanceCount = 'totalInstanceCount'
    totalPrimCount = 'totalPrimCount'
    untyped = 'untyped'
    usedLayerCount = 'usedLayerCount'
    pass
def AuthorCollection(*args, **kwargs) -> None:
    pass
def ComputeAllDependencies(*args, **kwargs) -> None:
    pass
def ComputeCollectionIncludesAndExcludes(*args, **kwargs) -> None:
    pass
def ComputeUsdStageStats(*args, **kwargs) -> None:
    pass
def CopyLayerMetadata(*args, **kwargs) -> None:
    pass
def CreateCollections(*args, **kwargs) -> None:
    pass
def CreateNewARKitUsdzPackage(*args, **kwargs) -> None:
    pass
def CreateNewUsdzPackage(*args, **kwargs) -> None:
    pass
def ExtractExternalReferences(*args, **kwargs) -> None:
    pass
def FlattenLayerStack(*args, **kwargs) -> None:
    pass
def FlattenLayerStackResolveAssetPath(*args, **kwargs) -> None:
    pass
def GenerateClipManifestName(*args, **kwargs) -> None:
    pass
def GenerateClipTopologyName(*args, **kwargs) -> None:
    pass
def GetAlphaAttributeNameForColor(*args, **kwargs) -> None:
    pass
def GetDirtyLayers(*args, **kwargs) -> None:
    pass
def GetMaterialsScopeName(*args, **kwargs) -> None:
    pass
def GetModelNameFromRootLayer(*args, **kwargs) -> None:
    pass
def GetPrefName(*args, **kwargs) -> None:
    pass
def GetPrimAtPathWithForwarding(*args, **kwargs) -> None:
    pass
def GetPrimaryCameraName(*args, **kwargs) -> None:
    pass
def GetPrimaryUVSetName(*args, **kwargs) -> None:
    pass
def GetRegisteredVariantSets(*args, **kwargs) -> None:
    pass
def LocalizeAsset(*args, **kwargs) -> None:
    pass
def ModifyAssetPaths(*args, **kwargs) -> None:
    pass
def StitchClips(*args, **kwargs) -> None:
    pass
def StitchClipsManifest(*args, **kwargs) -> None:
    pass
def StitchClipsTemplate(*args, **kwargs) -> None:
    pass
def StitchClipsTopology(*args, **kwargs) -> None:
    pass
def StitchInfo(*args, **kwargs) -> None:
    pass
def StitchLayers(*args, **kwargs) -> None:
    pass
def UninstancePrimAtPath(*args, **kwargs) -> None:
    pass
__MFB_FULL_PACKAGE_NAME = 'usdUtils'
