from __future__ import annotations
import pxr.Ndr._ndr
import typing
from pxr.Sdr import SdfTypeIndicator
import Boost.Python
import pxr.Ndr
import pxr.Tf

__all__ = [
    "DiscoveryPlugin",
    "DiscoveryPluginContext",
    "DiscoveryPluginList",
    "DiscoveryUri",
    "FsHelpersDiscoverFiles",
    "FsHelpersDiscoverNodes",
    "FsHelpersSplitShaderIdentifier",
    "Node",
    "NodeDiscoveryResult",
    "NodeList",
    "Property",
    "Registry",
    "SdfTypeIndicator",
    "Version",
    "VersionFilter",
    "VersionFilterAllVersions",
    "VersionFilterDefaultOnly"
]


class DiscoveryPlugin(Boost.Python.instance):
    @staticmethod
    def DiscoverNodes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSearchURIs(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class DiscoveryPluginContext(Boost.Python.instance):
    @staticmethod
    def GetSourceType(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class DiscoveryPluginList(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class DiscoveryUri(Boost.Python.instance):
    @property
    def resolvedUri(self) -> None:
        """
        :type: None
        """
    @property
    def uri(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 88
    pass
class Node(Boost.Python.instance):
    @staticmethod
    def GetContext(*args, **kwargs) -> None: ...
    @staticmethod
    def GetFamily(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInfoString(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInput(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInputNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMetadata(*args, **kwargs) -> None: ...
    @staticmethod
    def GetName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOutput(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOutputNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetResolvedDefinitionURI(*args, **kwargs) -> None: ...
    @staticmethod
    def GetResolvedImplementationURI(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSourceCode(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSourceType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVersion(*args, **kwargs) -> None: ...
    @staticmethod
    def IsValid(*args, **kwargs) -> None: ...
    pass
class NodeDiscoveryResult(Boost.Python.instance):
    @property
    def blindData(self) -> None:
        """
        :type: None
        """
    @property
    def discoveryType(self) -> None:
        """
        :type: None
        """
    @property
    def family(self) -> None:
        """
        :type: None
        """
    @property
    def identifier(self) -> None:
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
    def resolvedUri(self) -> None:
        """
        :type: None
        """
    @property
    def sourceCode(self) -> None:
        """
        :type: None
        """
    @property
    def sourceType(self) -> None:
        """
        :type: None
        """
    @property
    def subIdentifier(self) -> None:
        """
        :type: None
        """
    @property
    def uri(self) -> None:
        """
        :type: None
        """
    @property
    def version(self) -> None:
        """
        :type: None
        """
    pass
class NodeList(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class Property(Boost.Python.instance):
    @staticmethod
    def CanConnectTo(*args, **kwargs) -> None: ...
    @staticmethod
    def GetArraySize(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDefaultValue(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInfoString(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMetadata(*args, **kwargs) -> None: ...
    @staticmethod
    def GetName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTypeAsSdfType(*args, **kwargs) -> None: ...
    @staticmethod
    def IsArray(*args, **kwargs) -> None: ...
    @staticmethod
    def IsConnectable(*args, **kwargs) -> None: ...
    @staticmethod
    def IsDynamicArray(*args, **kwargs) -> None: ...
    @staticmethod
    def IsOutput(*args, **kwargs) -> None: ...
    pass
class Registry(Boost.Python.instance):
    @staticmethod
    def AddDiscoveryResult(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllNodeSourceTypes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeByIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeByIdentifierAndType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeByName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeByNameAndType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeFromAsset(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeFromSourceCode(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeIdentifiers(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodesByFamily(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodesByIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNodesByName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSearchURIs(*args, **kwargs) -> None: ...
    @staticmethod
    def SetExtraDiscoveryPlugins(*args, **kwargs) -> None: ...
    @staticmethod
    def SetExtraParserPlugins(*args, **kwargs) -> None: ...
    pass
class Version(Boost.Python.instance):
    @staticmethod
    def GetAsDefault(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMajor(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMinor(*args, **kwargs) -> None: ...
    @staticmethod
    def GetStringSuffix(*args, **kwargs) -> None: ...
    @staticmethod
    def IsDefault(*args, **kwargs) -> None: ...
    pass
class VersionFilter(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Ndr.VersionFilterDefaultOnly, Ndr.VersionFilterAllVersions)
    pass
class _AnnotatedBool(Boost.Python.instance):
    @property
    def message(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
class _FilesystemDiscoveryPlugin(DiscoveryPlugin, Boost.Python.instance):
    class Context(DiscoveryPluginContext, Boost.Python.instance):
        @property
        def expired(self) -> None:
            """
            True if this object has expired, False otherwise.

            :type: None
            """
        pass
    @staticmethod
    def DiscoverNodes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSearchURIs(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
def FsHelpersDiscoverFiles(*args, **kwargs) -> None:
    pass
def FsHelpersDiscoverNodes(*args, **kwargs) -> None:
    pass
def FsHelpersSplitShaderIdentifier(*args, **kwargs) -> None:
    pass
def _ValidateProperty(*args, **kwargs) -> None:
    pass
VersionFilterAllVersions: pxr.Ndr.VersionFilter # value = Ndr.VersionFilterAllVersions
VersionFilterDefaultOnly: pxr.Ndr.VersionFilter # value = Ndr.VersionFilterDefaultOnly
__MFB_FULL_PACKAGE_NAME = 'ndr'
