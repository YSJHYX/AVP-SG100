from __future__ import annotations
import pxr.UsdUI._usdUI
import typing
import Boost.Python
import pxr.Usd

__all__ = [
    "AccessibilityAPI",
    "Backdrop",
    "NodeGraphNodeAPI",
    "SceneGraphPrimAPI",
    "Tokens"
]


class AccessibilityAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def ApplyDefaultAPI(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDefaultAPI(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDescriptionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLabelAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreatePriorityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAll(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDescriptionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLabelAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPriorityAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def IsAccessibilityAPIPath(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class Backdrop(pxr.Usd.Typed, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def CreateDescriptionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDescriptionAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class NodeGraphNodeAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDisplayColorAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDocURIAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateExpansionStateAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateIconAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreatePosAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateSizeAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateStackingOrderAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDisplayColorAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDocURIAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetExpansionStateAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIconAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPosAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSizeAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetStackingOrderAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class SceneGraphPrimAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDisplayGroupAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDisplayNameAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDisplayGroupAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDisplayNameAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class Tokens(Boost.Python.instance):
    AccessibilityAPI = 'AccessibilityAPI'
    Backdrop = 'Backdrop'
    NodeGraphNodeAPI = 'NodeGraphNodeAPI'
    SceneGraphPrimAPI = 'SceneGraphPrimAPI'
    accessibility = 'accessibility'
    accessibility_MultipleApplyTemplate_Description = 'accessibility:__INSTANCE_NAME__:description'
    accessibility_MultipleApplyTemplate_Label = 'accessibility:__INSTANCE_NAME__:label'
    accessibility_MultipleApplyTemplate_Priority = 'accessibility:__INSTANCE_NAME__:priority'
    closed = 'closed'
    default_ = 'default'
    description = 'description'
    high = 'high'
    label = 'label'
    low = 'low'
    minimized = 'minimized'
    open = 'open'
    priority = 'priority'
    standard = 'standard'
    uiDescription = 'ui:description'
    uiDisplayGroup = 'ui:displayGroup'
    uiDisplayName = 'ui:displayName'
    uiNodegraphNodeDisplayColor = 'ui:nodegraph:node:displayColor'
    uiNodegraphNodeDocURI = 'ui:nodegraph:node:docURI'
    uiNodegraphNodeExpansionState = 'ui:nodegraph:node:expansionState'
    uiNodegraphNodeIcon = 'ui:nodegraph:node:icon'
    uiNodegraphNodePos = 'ui:nodegraph:node:pos'
    uiNodegraphNodeSize = 'ui:nodegraph:node:size'
    uiNodegraphNodeStackingOrder = 'ui:nodegraph:node:stackingOrder'
    pass
class _CanApplyResult(Boost.Python.instance):
    @property
    def whyNot(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
__MFB_FULL_PACKAGE_NAME = 'usdUI'
