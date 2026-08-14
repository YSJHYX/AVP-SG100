from __future__ import annotations
import pxr.UsdSemantics._usdSemantics
import typing
import Boost.Python
import pxr.Usd

__all__ = [
    "LabelsAPI",
    "LabelsQuery",
    "Tokens"
]


class LabelsAPI(pxr.Usd.APISchemaBase, pxr.Usd.SchemaBase, Boost.Python.instance):
    @staticmethod
    def Apply(*args, **kwargs) -> None: ...
    @staticmethod
    def CanApply(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeInheritedTaxonomies(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateLabelsAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAll(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDirectTaxonomies(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLabelsAttr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSchemaAttributeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def IsSemanticsLabelsAPIPath(*args, **kwargs) -> None: ...
    @staticmethod
    def _GetStaticTfType(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class LabelsQuery(Boost.Python.instance):
    @staticmethod
    def ComputeUniqueDirectLabels(*args, **kwargs) -> None: ...
    @staticmethod
    def ComputeUniqueInheritedLabels(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTaxonomy(*args, **kwargs) -> None: ...
    @staticmethod
    def HasDirectLabel(*args, **kwargs) -> None: ...
    @staticmethod
    def HasInheritedLabel(*args, **kwargs) -> None: ...
    pass
class Tokens(Boost.Python.instance):
    SemanticsLabelsAPI = 'SemanticsLabelsAPI'
    semanticsLabels = 'semantics:labels'
    semanticsLabels_MultipleApplyTemplate_ = 'semantics:labels:__INSTANCE_NAME__'
    pass
class _CanApplyResult(Boost.Python.instance):
    @property
    def whyNot(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
__MFB_FULL_PACKAGE_NAME = 'usdSemantics'
