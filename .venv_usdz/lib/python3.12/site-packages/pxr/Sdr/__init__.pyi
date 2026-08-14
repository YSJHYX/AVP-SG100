from __future__ import annotations
import pxr.Sdr._sdr
import typing
import Boost.Python
import pxr.Ndr
import pxr.Sdr
import pxr.Tf

__all__ = [
    "DiscoveryPlugin",
    "DiscoveryPluginContext",
    "DiscoveryUri",
    "FsHelpersDiscoverFiles",
    "FsHelpersDiscoverShaderNodes",
    "FsHelpersSplitShaderIdentifier",
    "NodeContext",
    "NodeDiscoveryResult",
    "NodeMetadata",
    "NodeRole",
    "PropertyMetadata",
    "PropertyRole",
    "PropertyTypes",
    "Registry",
    "SdfTypeIndicator",
    "ShaderNode",
    "ShaderNodeList",
    "ShaderProperty",
    "Version",
    "VersionFilter",
    "VersionFilterAllVersions",
    "VersionFilterDefaultOnly"
]


class DiscoveryPlugin(Boost.Python.instance):
    @staticmethod
    def DiscoverShaderNodes(*args, **kwargs) -> None: ...
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
class NodeContext(Boost.Python.instance):
    Displacement = 'displacement'
    DisplayFilter = 'displayFilter'
    Light = 'light'
    LightFilter = 'lightFilter'
    Pattern = 'pattern'
    PixelFilter = 'pixelFilter'
    SampleFilter = 'sampleFilter'
    Surface = 'surface'
    Volume = 'volume'
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
class NodeMetadata(Boost.Python.instance):
    Category = 'category'
    Departments = 'departments'
    Help = 'help'
    ImplementationName = '__SDR__implementationName'
    Label = 'label'
    Pages = 'pages'
    Primvars = 'primvars'
    Role = 'role'
    SdrDefinitionNameFallbackPrefix = 'sdrDefinitionNameFallbackPrefix'
    SdrUsdEncodingVersion = 'sdrUsdEncodingVersion'
    Target = '__SDR__target'
    pass
class NodeRole(Boost.Python.instance):
    Field = 'field'
    Math = 'math'
    Primvar = 'primvar'
    Texture = 'texture'
    pass
class PropertyMetadata(Boost.Python.instance):
    Colorspace = '__SDR__colorspace'
    Connectable = 'connectable'
    DefaultInput = '__SDR__defaultinput'
    Help = 'help'
    Hints = 'hints'
    ImplementationName = '__SDR__implementationName'
    IsAssetIdentifier = '__SDR__isAssetIdentifier'
    IsDynamicArray = 'isDynamicArray'
    Label = 'label'
    Options = 'options'
    Page = 'page'
    RenderType = 'renderType'
    Role = 'role'
    SdrUsdDefinitionType = 'sdrUsdDefinitionType'
    Tag = 'tag'
    Target = '__SDR__target'
    ValidConnectionTypes = 'validConnectionTypes'
    VstructConditionalExpr = 'vstructConditionalExpr'
    VstructMemberName = 'vstructMemberName'
    VstructMemberOf = 'vstructMemberOf'
    Widget = 'widget'
    pass
class PropertyRole(Boost.Python.instance):
    None = 'none'
    pass
class PropertyTypes(Boost.Python.instance):
    Color = 'color'
    Color4 = 'color4'
    Float = 'float'
    Int = 'int'
    Matrix = 'matrix'
    Normal = 'normal'
    Point = 'point'
    String = 'string'
    Struct = 'struct'
    Terminal = 'terminal'
    Unknown = 'unknown'
    Vector = 'vector'
    Vstruct = 'vstruct'
    pass
class Registry(Boost.Python.instance):
    @staticmethod
    def AddDiscoveryResult(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllShaderNodeSourceTypes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodeByIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodeByIdentifierAndType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodeByName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodeByNameAndType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodeFromAsset(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodeFromSourceCode(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodeIdentifiers(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodeNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodesByFamily(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodesByIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderNodesByName(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class SdfTypeIndicator(Boost.Python.instance):
    @staticmethod
    def GetNdrType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSdfType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetSdrType(*args, **kwargs) -> None: ...
    @staticmethod
    def HasSdfType(*args, **kwargs) -> None: ...
    pass
class ShaderNode(pxr.Ndr.Node, Boost.Python.instance):
    @staticmethod
    def GetAdditionalPrimvarProperties(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllVstructNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAssetIdentifierInputNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCategory(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDefaultInput(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDepartments(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHelp(*args, **kwargs) -> None: ...
    @staticmethod
    def GetImplementationName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLabel(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPages(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrimvars(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPropertyNamesForPage(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRole(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderInput(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderInputNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderOutput(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderOutputNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetShaderVersion(*args, **kwargs) -> None: ...
    pass
class ShaderNodeList(Boost.Python.instance):
    @staticmethod
    def append(*args, **kwargs) -> None: ...
    @staticmethod
    def extend(*args, **kwargs) -> None: ...
    __instance_size__ = 48
    pass
class ShaderProperty(pxr.Ndr.Property, Boost.Python.instance):
    @staticmethod
    def CanConnectTo(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDefaultValueAsSdfType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHelp(*args, **kwargs) -> None: ...
    @staticmethod
    def GetHints(*args, **kwargs) -> None: ...
    @staticmethod
    def GetImplementationName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLabel(*args, **kwargs) -> None: ...
    @staticmethod
    def GetOptions(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPage(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVStructConditionalExpr(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVStructMemberName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVStructMemberOf(*args, **kwargs) -> None: ...
    @staticmethod
    def GetValidConnectionTypes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetWidget(*args, **kwargs) -> None: ...
    @staticmethod
    def IsAssetIdentifier(*args, **kwargs) -> None: ...
    @staticmethod
    def IsDefaultInput(*args, **kwargs) -> None: ...
    @staticmethod
    def IsVStruct(*args, **kwargs) -> None: ...
    @staticmethod
    def IsVStructMember(*args, **kwargs) -> None: ...
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
    allValues: tuple # value = (Sdr.VersionFilterDefaultOnly, Sdr.VersionFilterAllVersions)
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
    def DiscoverShaderNodes(*args, **kwargs) -> None: ...
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
def FsHelpersDiscoverShaderNodes(*args, **kwargs) -> None:
    pass
def FsHelpersSplitShaderIdentifier(*args, **kwargs) -> None:
    pass
VersionFilterAllVersions: pxr.Sdr.VersionFilter # value = Sdr.VersionFilterAllVersions
VersionFilterDefaultOnly: pxr.Sdr.VersionFilter # value = Sdr.VersionFilterDefaultOnly
__MFB_FULL_PACKAGE_NAME = 'sdr'
