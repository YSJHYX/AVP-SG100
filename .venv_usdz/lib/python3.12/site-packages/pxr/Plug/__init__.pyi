from __future__ import annotations
import pxr.Plug._plug
import typing
import Boost.Python

__all__ = [
    "Notice",
    "Plugin",
    "Registry"
]


class Notice(Boost.Python.instance):
    class Base(pxr.Tf.Notice, Boost.Python.instance):
        pass
    class DidRegisterPlugins(Base, pxr.Tf.Notice, Boost.Python.instance):
        @staticmethod
        def GetNewPlugins(*args, **kwargs) -> None: ...
        pass
    pass
class Plugin(Boost.Python.instance):
    @staticmethod
    def DeclaresType(*args, **kwargs) -> None: ...
    @staticmethod
    def FindPluginResource(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMetadataForType(*args, **kwargs) -> None: ...
    @staticmethod
    def Load(*args, **kwargs) -> None: ...
    @staticmethod
    def MakeResourcePath(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    @property
    def isLoaded(self) -> None:
        """
        :type: None
        """
    @property
    def isPythonModule(self) -> None:
        """
        :type: None
        """
    @property
    def isResource(self) -> None:
        """
        :type: None
        """
    @property
    def metadata(self) -> None:
        """
        :type: None
        """
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
    @property
    def resourcePath(self) -> None:
        """
        :type: None
        """
    pass
class Registry(Boost.Python.instance):
    @staticmethod
    def FindDerivedTypeByName(*args, **kwargs) -> None: ...
    @staticmethod
    def FindTypeByName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllDerivedTypes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllPlugins(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDirectlyDerivedTypes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPluginForType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPluginWithName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetStringFromPluginMetaData(*args, **kwargs) -> None: ...
    @staticmethod
    def RegisterPlugins(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class _TestPlugBase1(Boost.Python.instance):
    @staticmethod
    def GetTypeName(*args, **kwargs) -> None: ...
    @staticmethod
    def TestAcceptPluginSequence(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class _TestPlugBase2(Boost.Python.instance):
    @staticmethod
    def GetTypeName(*args, **kwargs) -> None: ...
    @staticmethod
    def TestAcceptPluginSequence(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class _TestPlugBase3(Boost.Python.instance):
    @staticmethod
    def GetTypeName(*args, **kwargs) -> None: ...
    @staticmethod
    def TestAcceptPluginSequence(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class _TestPlugBase4(Boost.Python.instance):
    @staticmethod
    def GetTypeName(*args, **kwargs) -> None: ...
    @staticmethod
    def TestAcceptPluginSequence(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
def _LoadPluginsConcurrently(*args, **kwargs) -> None:
    pass
__MFB_FULL_PACKAGE_NAME = 'plug'
