from __future__ import annotations
import pxr.Ar._ar
import typing
import Boost.Python
import pxr.Ar

__all__ = [
    "Ar_PyAsset",
    "AssetInfo",
    "DefaultResolver",
    "DefaultResolverContext",
    "GetRegisteredURISchemes",
    "GetResolver",
    "GetUnderlyingResolver",
    "IsPackageRelativePath",
    "JoinPackageRelativePath",
    "Notice",
    "ResolvedPath",
    "Resolver",
    "ResolverContext",
    "ResolverContextBinder",
    "ResolverScopedCache",
    "SetPreferredResolver",
    "SplitPackageRelativePathInner",
    "SplitPackageRelativePathOuter",
    "Timestamp"
]


class Ar_PyAsset(Boost.Python.instance):
    @staticmethod
    def GetBuffer(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSize(*args, **kwargs) -> None: ...
    @staticmethod
    def Read(*args, **kwargs) -> None: ...
    pass
class AssetInfo(Boost.Python.instance):
    @property
    def assetName(self) -> None:
        """
        :type: None
        """
    @property
    def resolverInfo(self) -> None:
        """
        :type: None
        """
    @property
    def version(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 136
    pass
class DefaultResolver(Resolver, Boost.Python.instance):
    @staticmethod
    def SetDefaultSearchPath(*args, **kwargs) -> None: ...
    pass
class DefaultResolverContext(Boost.Python.instance):
    @staticmethod
    def GetSearchPath(*args, **kwargs) -> None: ...
    pass
class Notice(Boost.Python.instance):
    class ResolverChanged(ResolverNotice, pxr.Tf.Notice, Boost.Python.instance):
        @staticmethod
        def AffectsContext(*args, **kwargs) -> None: ...
        pass
    class ResolverNotice(pxr.Tf.Notice, Boost.Python.instance):
        pass
    pass
class ResolvedPath(Boost.Python.instance):
    @staticmethod
    def GetPathString(*args, **kwargs) -> None: ...
    __instance_size__ = 56
    pass
class Resolver(Boost.Python.instance):
    @staticmethod
    def CanWriteAssetToPath(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateContextFromString(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateContextFromStrings(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDefaultContext(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateDefaultContextForAsset(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def CreateIdentifierForNewAsset(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAssetInfo(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCurrentContext(*args, **kwargs) -> None: ...
    @staticmethod
    def GetExtension(*args, **kwargs) -> None: ...
    @staticmethod
    def GetModificationTimestamp(*args, **kwargs) -> None: ...
    @staticmethod
    def IsContextDependentPath(*args, **kwargs) -> None: ...
    @staticmethod
    def OpenAsset(*args, **kwargs) -> None: ...
    @staticmethod
    def RefreshContext(*args, **kwargs) -> None: ...
    @staticmethod
    def Resolve(*args, **kwargs) -> None: ...
    @staticmethod
    def ResolveForNewAsset(*args, **kwargs) -> None: ...
    pass
class ResolverContext(Boost.Python.instance):
    @staticmethod
    def Get(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDebugString(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    pass
class ResolverContextBinder(Boost.Python.instance):
    __instance_size__ = 56
    pass
class ResolverScopedCache(Boost.Python.instance):
    __instance_size__ = 32
    pass
class Timestamp(Boost.Python.instance):
    @staticmethod
    def GetTime(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValid(*args, **kwargs) -> None: ...
    __instance_size__ = 32
    pass
class _PyAnnotatedBoolResult(Boost.Python.instance):
    @property
    def whyNot(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
def GetRegisteredURISchemes(*args, **kwargs) -> None:
    pass
def GetResolver(*args, **kwargs) -> None:
    pass
def GetUnderlyingResolver(*args, **kwargs) -> None:
    pass
def IsPackageRelativePath(*args, **kwargs) -> None:
    pass
def JoinPackageRelativePath(*args, **kwargs) -> None:
    pass
def SetPreferredResolver(*args, **kwargs) -> None:
    pass
def SplitPackageRelativePathInner(*args, **kwargs) -> None:
    pass
def SplitPackageRelativePathOuter(*args, **kwargs) -> None:
    pass
def _TestImplicitConversion(*args, **kwargs) -> None:
    pass
__MFB_FULL_PACKAGE_NAME = 'ar'
