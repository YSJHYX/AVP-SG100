from __future__ import annotations
import pxr.Kind._kind
import typing
import Boost.Python

__all__ = [
    "Registry",
    "Tokens"
]


class Registry(Boost.Python.instance):
    @staticmethod
    def GetAllKinds(*args, **kwargs) -> None: ...
    @staticmethod
    def GetBaseKind(*args, **kwargs) -> None: ...
    @staticmethod
    def HasKind(*args, **kwargs) -> None: ...
    @staticmethod
    def IsA(*args, **kwargs) -> None: ...
    @staticmethod
    def IsAssembly(*args, **kwargs) -> None: ...
    @staticmethod
    def IsComponent(*args, **kwargs) -> None: ...
    @staticmethod
    def IsGroup(*args, **kwargs) -> None: ...
    @staticmethod
    def IsModel(*args, **kwargs) -> None: ...
    @staticmethod
    def IsSubComponent(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class Tokens(Boost.Python.instance):
    assembly = 'assembly'
    component = 'component'
    group = 'group'
    model = 'model'
    subcomponent = 'subcomponent'
    pass
__MFB_FULL_PACKAGE_NAME = 'kind'
