from __future__ import annotations
import pxr.Tf._tf
import typing
import Boost.Python
import pxr.Tf

__all__ = [
    "CallContext",
    "CppException",
    "Debug",
    "DiagnosticType",
    "DictionaryStrcmp",
    "DumpTokenStats",
    "Enum",
    "Error",
    "FindLongestAccessiblePrefix",
    "GetAppLaunchTime",
    "GetCurrentScopeDescriptionStack",
    "GetEnvSetting",
    "GetStackTrace",
    "InstallTerminateAndCrashHandlers",
    "IsValidIdentifier",
    "LogStackTrace",
    "MakeValidIdentifier",
    "MallocTag",
    "Notice",
    "PrintStackTrace",
    "PyModuleWasLoaded",
    "RealPath",
    "RefPtrTracker",
    "ReportActiveErrorMarks",
    "RepostErrors",
    "ScopeDescription",
    "ScriptModuleLoader",
    "SetPythonExceptionDebugTracingEnabled",
    "Singleton",
    "StatusObject",
    "Stopwatch",
    "StringSplit",
    "StringToDouble",
    "StringToLong",
    "StringToULong",
    "TF_APPLICATION_EXIT_TYPE",
    "TF_DIAGNOSTIC_CODING_ERROR_TYPE",
    "TF_DIAGNOSTIC_FATAL_CODING_ERROR_TYPE",
    "TF_DIAGNOSTIC_FATAL_ERROR_TYPE",
    "TF_DIAGNOSTIC_NONFATAL_ERROR_TYPE",
    "TF_DIAGNOSTIC_RUNTIME_ERROR_TYPE",
    "TF_DIAGNOSTIC_STATUS_TYPE",
    "TF_DIAGNOSTIC_WARNING_TYPE",
    "TemplateString",
    "Tf_PyEnumWrapper",
    "Tf_TestAnnotatedBoolResult",
    "Tf_TestPyContainerConversions",
    "Tf_TestPyOptionalStd",
    "TouchFile",
    "Type",
    "Warning"
]


class CallContext(Boost.Python.instance):
    @property
    def file(self) -> None:
        """
        :type: None
        """
    @property
    def function(self) -> None:
        """
        :type: None
        """
    @property
    def line(self) -> None:
        """
        :type: None
        """
    @property
    def prettyFunction(self) -> None:
        """
        :type: None
        """
    pass
class CppException(Exception, BaseException):
    pass
class Debug(Boost.Python.instance):
    @staticmethod
    def GetDebugSymbolDescription(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDebugSymbolDescriptions(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDebugSymbolNames(*args, **kwargs) -> None: ...
    @staticmethod
    def IsDebugSymbolNameEnabled(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDebugSymbolsByName(*args, **kwargs) -> None: ...
    @staticmethod
    def SetOutputFile(*args, **kwargs) -> None: ...
    pass
class DiagnosticType(Tf_PyEnumWrapper, Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Tf.TF_DIAGNOSTIC_CODING_ERROR_TYPE, Tf.TF_DIAGNOSTIC_FATAL_CODING_ERROR_TYPE, Tf.TF_DIAGNOSTIC_RUNTIME_ERROR_TYPE, Tf.TF_DIAGNOSTIC_FATAL_ERROR_TYPE, Tf.TF_DIAGNOSTIC_NONFATAL_ERROR_TYPE, Tf.TF_DIAGNOSTIC_WARNING_TYPE, Tf.TF_DIAGNOSTIC_STATUS_TYPE, Tf.TF_APPLICATION_EXIT_TYPE)
    pass
class Enum(Boost.Python.instance):
    @staticmethod
    def GetValueFromFullName(*args, **kwargs) -> None: ...
    pass
class Error(_DiagnosticBase, Boost.Python.instance):
    class Mark(Boost.Python.instance):
        @staticmethod
        def Clear(*args, **kwargs) -> None: ...
        @staticmethod
        def GetErrors(*args, **kwargs) -> None: 
            """
            A list of the errors held by this mark.
            """
        @staticmethod
        def IsClean(*args, **kwargs) -> None: ...
        @staticmethod
        def SetMark(*args, **kwargs) -> None: ...
        __instance_size__ = 32
        pass
    @property
    def errorCode(self) -> None:
        """
        The error code posted for this error.

        :type: None
        """
    @property
    def errorCodeString(self) -> None:
        """
        The error code posted for this error, as a string.

        :type: None
        """
    pass
class MallocTag(Boost.Python.instance):
    class CallTree(Boost.Python.instance):
        class CallSite(Boost.Python.instance):
            @property
            def nBytes(self) -> None:
                """
                :type: None
                """
            @property
            def name(self) -> None:
                """
                :type: None
                """
            pass
        class PathNode(Boost.Python.instance):
            @staticmethod
            def GetChildren(*args, **kwargs) -> None: ...
            @property
            def nAllocations(self) -> None:
                """
                :type: None
                """
            @property
            def nBytes(self) -> None:
                """
                :type: None
                """
            @property
            def nBytesDirect(self) -> None:
                """
                :type: None
                """
            @property
            def siteName(self) -> None:
                """
                :type: None
                """
            pass
        @staticmethod
        def GetCallSites(*args, **kwargs) -> None: ...
        @staticmethod
        def GetPrettyPrintString(*args, **kwargs) -> None: ...
        @staticmethod
        def GetRoot(*args, **kwargs) -> None: ...
        @staticmethod
        def LoadReport(*args, **kwargs) -> None: ...
        @staticmethod
        def LogReport(*args, **kwargs) -> None: ...
        @staticmethod
        def Report(*args, **kwargs) -> None: ...
        pass
    @staticmethod
    def GetCallStacks(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCallTree(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMaxTotalBytes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTotalBytes(*args, **kwargs) -> None: ...
    @staticmethod
    def Initialize(*args, **kwargs) -> None: ...
    @staticmethod
    def IsInitialized(*args, **kwargs) -> None: ...
    @staticmethod
    def SetCapturedMallocStacksMatchList(*args, **kwargs) -> None: ...
    @staticmethod
    def SetDebugMatchList(*args, **kwargs) -> None: ...
    pass
class PyModuleWasLoaded(Notice, Boost.Python.instance):
    @staticmethod
    def name(*args, **kwargs) -> None: ...
    pass
class Notice(Boost.Python.instance):
    class Listener(Boost.Python.instance):
        """
        Represents the Notice connection between senders and receivers of notices.  When a Listener object expires the connection is broken. You can also use the Revoke() function to break the connection. A Listener object is returned from the Register() and  RegisterGlobally() functions. 
        """
        @staticmethod
        def Revoke(*args, **kwargs) -> None: 
            """
            Revoke interest by a notice listener.  This function revokes interest in the particular notice type and call-back method that its Listener object was registered for. 
            """
        pass
    @staticmethod
    def Register( noticeType, callback, sender ) -> Listener : 
        """
        noticeType : Tf.Notice
        callback : function
        sender : object

        Register a listener as being interested in a TfNotice  type from a specific sender.  Notice listener will get sender  as an argument.     Registration of interest in a notice class N automatically  registers interest in all classes derived from N.  When a  notice of appropriate type is received, the listening object's  member-function method is called with the notice.     To reverse the registration, call Revoke() on the Listener object returned by this call. 

        noticeType : Tf.Notice
        callback : function
        sender : object

        Register a listener as being interested in a TfNotice  type from a specific sender.  Notice listener will get sender  as an argument.     Registration of interest in a notice class N automatically  registers interest in all classes derived from N.  When a  notice of appropriate type is received, the listening object's  member-function method is called with the notice.     To reverse the registration, call Revoke() on the Listener object returned by this call. 
        """
    @staticmethod
    def RegisterGlobally( noticeType, callback ) -> Listener : 
        """
        noticeType : Tf.Notice
        callback : function

        Register a listener as being interested in a TfNotice type from any sender.  The notice listener does not get sender as an argument. 
        """
    @staticmethod
    def Send(*args, **kwargs) -> None: 
        """
        sender : object 

        Deliver the notice to interested listeners, returning the number of interested listeners. This is the recommended form of Send.  It takes the sender as an argument. Listeners that registered for the given sender AND listeners that registered globally will get the notice. 

        sender : object 

        Deliver the notice to interested listeners, returning the number of interested listeners. This is the recommended form of Send.  It takes the sender as an argument. Listeners that registered for the given sender AND listeners that registered globally will get the notice. 
        """
    @staticmethod
    def SendGlobally(*args, **kwargs) -> None: 
        """
        Deliver the notice to interested listeners.   For most clients it is recommended to use the Send(sender) version of Send() rather than this one.  Clients that use this form of Send will prevent listeners from being able to register to receive notices based on the sender of the notice. ONLY listeners that registered globally will get the notice. 
        """
    pass
class RefPtrTracker(Boost.Python.instance):
    @staticmethod
    def GetAllTracesReport(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllWatchedCountsReport(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTracesReportForWatched(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class ScopeDescription(Boost.Python.instance):
    @staticmethod
    def SetDescription(*args, **kwargs) -> None: ...
    __instance_size__ = 64
    pass
class ScriptModuleLoader(Boost.Python.instance):
    @staticmethod
    def GetModulesDict(*args, **kwargs) -> None: ...
    @staticmethod
    def WriteDotFile(*args, **kwargs) -> None: ...
    @staticmethod
    def _LoadModulesForLibrary(*args, **kwargs) -> None: ...
    @staticmethod
    def _RegisterLibrary(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class Singleton(Boost.Python.instance):
    pass
class StatusObject(_DiagnosticBase, Boost.Python.instance):
    pass
class Stopwatch(Boost.Python.instance):
    @staticmethod
    def AddFrom(*args, **kwargs) -> None: ...
    @staticmethod
    def Reset(*args, **kwargs) -> None: ...
    @staticmethod
    def Start(*args, **kwargs) -> None: ...
    @staticmethod
    def Stop(*args, **kwargs) -> None: ...
    @property
    def microseconds(self) -> None:
        """
        :type: None
        """
    @property
    def milliseconds(self) -> None:
        """
        :type: None
        """
    @property
    def nanoseconds(self) -> None:
        """
        :type: None
        """
    @property
    def sampleCount(self) -> None:
        """
        :type: None
        """
    @property
    def seconds(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 48
    pass
class TemplateString(Boost.Python.instance):
    @staticmethod
    def GetEmptyMapping(*args, **kwargs) -> None: ...
    @staticmethod
    def GetParseErrors(*args, **kwargs) -> None: ...
    @staticmethod
    def SafeSubstitute(*args, **kwargs) -> None: ...
    @staticmethod
    def Substitute(*args, **kwargs) -> None: ...
    @property
    def template(self) -> None:
        """
        :type: None
        """
    @property
    def valid(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class Tf_PyEnumWrapper(Enum, Boost.Python.instance):
    @property
    def displayName(self) -> None:
        """
        :type: None
        """
    @property
    def fullName(self) -> None:
        """
        :type: None
        """
    @property
    def name(self) -> None:
        """
        :type: None
        """
    @property
    def value(self) -> None:
        """
        :type: None
        """
    pass
class Tf_TestAnnotatedBoolResult(Boost.Python.instance):
    @property
    def annotation(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 64
    pass
class Tf_TestPyContainerConversions(Boost.Python.instance):
    @staticmethod
    def GetPairTimesTwo(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTokens(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTupleTimesTwo(*args, **kwargs) -> None: ...
    @staticmethod
    def GetVectorTimesTwo(*args, **kwargs) -> None: ...
    @staticmethod
    def Init(*args, **kwargs) -> None: ...
    __instance_size__ = 32
    pass
class Tf_TestPyOptionalStd(Boost.Python.instance):
    @staticmethod
    def TakesOptional(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalChar(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalDouble(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalFloat(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalInt(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalLong(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalShort(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalString(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalStringVector(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalUChar(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalUInt(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalULong(*args, **kwargs) -> None: ...
    @staticmethod
    def TestOptionalUShort(*args, **kwargs) -> None: ...
    __instance_size__ = 32
    pass
class Type(Boost.Python.instance):
    @staticmethod
    def AddAlias(*args, **kwargs) -> None: ...
    @staticmethod
    def Define(*args, **kwargs) -> None: ...
    @staticmethod
    def Find(*args, **kwargs) -> None: ...
    @staticmethod
    def FindByName(*args, **kwargs) -> None: ...
    @staticmethod
    def FindDerivedByName(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAliases(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllAncestorTypes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAllDerivedTypes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRoot(*args, **kwargs) -> None: ...
    @staticmethod
    def IsA(*args, **kwargs) -> None: ...
    @staticmethod
    def _DumpTypeHierarchy(*args, **kwargs) -> None: ...
    @property
    def baseTypes(self) -> None:
        """
        :type: None
        """
    @property
    def derivedTypes(self) -> None:
        """
        :type: None
        """
    @property
    def isEnumType(self) -> None:
        """
        :type: None
        """
    @property
    def isPlainOldDataType(self) -> None:
        """
        :type: None
        """
    @property
    def isUnknown(self) -> None:
        """
        :type: None
        """
    @property
    def pythonClass(self) -> None:
        """
        :type: None
        """
    @property
    def sizeof(self) -> None:
        """
        :type: None
        """
    @property
    def typeName(self) -> None:
        """
        :type: None
        """
    Unknown: pxr.Tf.Type # value = Tf.Type.Unknown
    __instance_size__ = 32
    pass
class Warning(_DiagnosticBase, Boost.Python.instance):
    pass
class _ClassWithClassMethod(Boost.Python.instance):
    @staticmethod
    def Test(*args, **kwargs) -> None: ...
    __instance_size__ = 32
    pass
class _ClassWithVarArgInit(Boost.Python.instance):
    @property
    def allowExtraArgs(self) -> None:
        """
        :type: None
        """
    @property
    def args(self) -> None:
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
    def kwargs(self) -> None:
        """
        :type: None
        """
    pass
class _DiagnosticBase(Boost.Python.instance):
    @property
    def commentary(self) -> None:
        """
        The commentary string describing this error.

        :type: None
        """
    @property
    def diagnosticCode(self) -> None:
        """
        The diagnostic code posted.

        :type: None
        """
    @property
    def diagnosticCodeString(self) -> None:
        """
        The error code posted for this error, as a string.

        :type: None
        """
    @property
    def sourceFileName(self) -> None:
        """
        The source file name that the error was posted from.

        :type: None
        """
    @property
    def sourceFunction(self) -> None:
        """
        The source function that the error was posted from.

        :type: None
        """
    @property
    def sourceLineNumber(self) -> None:
        """
        The source line number that the error was posted from.

        :type: None
        """
    pass
class _Enum(Boost.Python.instance):
    class TestEnum2(Tf_PyEnumWrapper, Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = '_Enum'
        allValues: tuple # value = (Tf._Enum.One, Tf._Enum.Two, Tf._Enum.Three)
        pass
    class TestKeywords(Tf_PyEnumWrapper, Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        False_: pxr.Tf.TestKeywords # value = Tf._Enum.TestKeywords.False_
        None_: pxr.Tf.TestKeywords # value = Tf._Enum.TestKeywords.None_
        True_: pxr.Tf.TestKeywords # value = Tf._Enum.TestKeywords.True_
        _baseName = '_Enum.TestKeywords'
        allValues: tuple # value = (Tf._Enum.TestKeywords.None_, Tf._Enum.TestKeywords.False_, Tf._Enum.TestKeywords.True_, Tf._Enum.TestKeywords.print_, Tf._Enum.TestKeywords.import_, Tf._Enum.TestKeywords.global_)
        global_: pxr.Tf.TestKeywords # value = Tf._Enum.TestKeywords.global_
        import_: pxr.Tf.TestKeywords # value = Tf._Enum.TestKeywords.import_
        print_: pxr.Tf.TestKeywords # value = Tf._Enum.TestKeywords.print_
        pass
    class TestScopedEnum(Tf_PyEnumWrapper, Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        Alef: pxr.Tf.TestScopedEnum # value = Tf._Enum.TestScopedEnum.Alef
        Bet: pxr.Tf.TestScopedEnum # value = Tf._Enum.TestScopedEnum.Bet
        Gimel: pxr.Tf.TestScopedEnum # value = Tf._Enum.TestScopedEnum.Gimel
        _baseName = '_Enum.TestScopedEnum'
        allValues: tuple # value = (Tf._Enum.TestScopedEnum.Alef, Tf._Enum.TestScopedEnum.Bet, Tf._Enum.TestScopedEnum.Gimel)
        pass
    One: pxr.Tf.TestEnum2 # value = Tf._Enum.One
    Three: pxr.Tf.TestEnum2 # value = Tf._Enum.Three
    Two: pxr.Tf.TestEnum2 # value = Tf._Enum.Two
    pass
class _TestDerived(_TestBase, Boost.Python.instance):
    @staticmethod
    def Virtual(*args, **kwargs) -> None: ...
    @staticmethod
    def Virtual2(*args, **kwargs) -> None: ...
    @staticmethod
    def Virtual3(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class _TestBase(Boost.Python.instance):
    @staticmethod
    def TestCallVirtual(*args, **kwargs) -> None: ...
    @staticmethod
    def Virtual(*args, **kwargs) -> None: ...
    @staticmethod
    def Virtual2(*args, **kwargs) -> None: ...
    @staticmethod
    def Virtual3(*args, **kwargs) -> None: ...
    @staticmethod
    def Virtual4(*args, **kwargs) -> None: ...
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    pass
class _TestEnum(Tf_PyEnumWrapper, Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Tf._Alpha, Tf._Bravo, Tf._Charlie, Tf._Delta)
    pass
class _TestErrorClass1(Boost.Python.instance):
    class _TestErrorClass2(Boost.Python.instance):
        @staticmethod
        def ClassMethod(*args, **kwargs) -> None: ...
        @staticmethod
        def Create(*args, **kwargs) -> None: ...
        @staticmethod
        def InstanceMethod(*args, **kwargs) -> None: ...
        @staticmethod
        def StaticMethod(*args, **kwargs) -> None: ...
        @property
        def property(self) -> None:
            """
            :type: None
            """
        @property
        def property_2(self) -> None:
            """
            :type: None
            """
        __instance_size__ = 32
        static_property = 'StaticGetter'
        static_property_2 = 'StaticGetter'
        pass
    @staticmethod
    def ClassMethod(*args, **kwargs) -> None: ...
    @staticmethod
    def Create(*args, **kwargs) -> None: ...
    @staticmethod
    def InstanceMethod(*args, **kwargs) -> None: ...
    @staticmethod
    def StaticMethod(*args, **kwargs) -> None: ...
    @staticmethod
    def _TestErrorFunction(*args, **kwargs) -> None: ...
    @property
    def property(self) -> None:
        """
        :type: None
        """
    @property
    def property_2(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 32
    static_property = 'StaticGetter'
    static_property_2 = 'StaticGetter'
    pass
class _TestScopedEnum(Tf_PyEnumWrapper, Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    Beryllium: pxr.Tf._TestScopedEnum # value = Tf._TestScopedEnum.Beryllium
    Boron: pxr.Tf._TestScopedEnum # value = Tf._TestScopedEnum.Boron
    Hydrogen: pxr.Tf._TestScopedEnum # value = Tf._TestScopedEnum.Hydrogen
    Lithium: pxr.Tf._TestScopedEnum # value = Tf._TestScopedEnum.Lithium
    _baseName = '_TestScopedEnum'
    allValues: tuple # value = (Tf._TestScopedEnum.Hydrogen, Tf._TestScopedEnum.Lithium, Tf._TestScopedEnum.Beryllium, Tf._TestScopedEnum.Boron)
    pass
class _TestStaticTokens(Boost.Python.instance):
    orange = 'orange'
    pear = "d'Anjou"
    pass
class _testStaticTokens(Boost.Python.instance):
    orange = 'orange'
    pear = "d'Anjou"
    pass
def DictionaryStrcmp(*args, **kwargs) -> None:
    pass
def DumpTokenStats(*args, **kwargs) -> None:
    pass
def FindLongestAccessiblePrefix(*args, **kwargs) -> None:
    pass
def GetAppLaunchTime() -> int :
    """
    Return the time (in seconds since the epoch) at which the application was started.
    """
def GetCurrentScopeDescriptionStack(*args, **kwargs) -> None:
    pass
def GetEnvSetting(*args, **kwargs) -> None:
    pass
def GetStackTrace(*args, **kwargs) -> None:
    """
    Return both the C++ and the python stack as a string.
    """
def InstallTerminateAndCrashHandlers(*args, **kwargs) -> None:
    pass
def IsValidIdentifier(*args, **kwargs) -> None:
    pass
def LogStackTrace(*args, **kwargs) -> None:
    pass
def MakeValidIdentifier(*args, **kwargs) -> None:
    pass
def PrintStackTrace(*args, **kwargs) -> None:
    """
    Prints both the C++ and the python stack to the file provided.
    """
def RealPath(*args, **kwargs) -> None:
    pass
def ReportActiveErrorMarks(*args, **kwargs) -> None:
    pass
def RepostErrors(*args, **kwargs) -> None:
    pass
def SetPythonExceptionDebugTracingEnabled(*args, **kwargs) -> None:
    pass
def StringSplit(*args, **kwargs) -> None:
    pass
def StringToDouble(*args, **kwargs) -> None:
    pass
def StringToLong(*args, **kwargs) -> None:
    pass
def StringToULong(*args, **kwargs) -> None:
    pass
def TouchFile(*args, **kwargs) -> None:
    pass
def _CallThrowTest(*args, **kwargs) -> None:
    pass
def _ConvertByteListToByteArray(*args, **kwargs) -> None:
    pass
def _DerivedFactory(*args, **kwargs) -> None:
    pass
def _DerivedNullFactory(*args, **kwargs) -> None:
    pass
def _Fatal(*args, **kwargs) -> None:
    pass
def _GetLongMax(*args, **kwargs) -> None:
    pass
def _GetLongMin(*args, **kwargs) -> None:
    pass
def _GetULongMax(*args, **kwargs) -> None:
    pass
def _RaiseCodingError(*args, **kwargs) -> None:
    pass
def _RaiseRuntimeError(*args, **kwargs) -> None:
    pass
def _ReturnsBase(*args, **kwargs) -> None:
    pass
def _ReturnsBaseRefPtr(*args, **kwargs) -> None:
    pass
def _ReturnsConstBase(*args, **kwargs) -> None:
    pass
def _RoundTripWrapperCallTest(*args, **kwargs) -> None:
    pass
def _RoundTripWrapperIndexTest(*args, **kwargs) -> None:
    pass
def _RoundTripWrapperTest(*args, **kwargs) -> None:
    pass
def _Status(*args, **kwargs) -> None:
    pass
def _TakesBase(*args, **kwargs) -> None:
    pass
def _TakesConstBase(*args, **kwargs) -> None:
    pass
def _TakesDerived(*args, **kwargs) -> None:
    pass
def _TakesReference(*args, **kwargs) -> None:
    pass
def _TakesVecVecString(*args, **kwargs) -> None:
    pass
def _TestAnnotatedBoolResult(*args, **kwargs) -> None:
    pass
def _TestErrorFunction(*args, **kwargs) -> None:
    pass
def _ThrowCppException(*args, **kwargs) -> None:
    pass
def _ThrowTest(*args, **kwargs) -> None:
    pass
def _Warn(*args, **kwargs) -> None:
    pass
def __SetErrorExceptionClass(*args, **kwargs) -> None:
    pass
def _callUnboundInstance(*args, **kwargs) -> None:
    pass
def _callback(*args, **kwargs) -> None:
    pass
def _doErrors(*args, **kwargs) -> None:
    pass
def _invokeTestCallback(*args, **kwargs) -> None:
    pass
def _mightRaise(*args, **kwargs) -> None:
    pass
def _registerInvalidEnum(*args, **kwargs) -> None:
    pass
def _returnsTfEnum(*args, **kwargs) -> None:
    pass
def _sendTfNoticeWithSender(*args, **kwargs) -> None:
    pass
def _setTestCallback(*args, **kwargs) -> None:
    pass
def _stringCallback(*args, **kwargs) -> None:
    pass
def _stringStringCallback(*args, **kwargs) -> None:
    pass
def _takesTestEnum(*args, **kwargs) -> None:
    pass
def _takesTestEnum2(*args, **kwargs) -> None:
    pass
def _takesTfEnum(*args, **kwargs) -> None:
    pass
TF_APPLICATION_EXIT_TYPE: pxr.Tf.DiagnosticType # value = Tf.TF_APPLICATION_EXIT_TYPE
TF_DIAGNOSTIC_CODING_ERROR_TYPE: pxr.Tf.DiagnosticType # value = Tf.TF_DIAGNOSTIC_CODING_ERROR_TYPE
TF_DIAGNOSTIC_FATAL_CODING_ERROR_TYPE: pxr.Tf.DiagnosticType # value = Tf.TF_DIAGNOSTIC_FATAL_CODING_ERROR_TYPE
TF_DIAGNOSTIC_FATAL_ERROR_TYPE: pxr.Tf.DiagnosticType # value = Tf.TF_DIAGNOSTIC_FATAL_ERROR_TYPE
TF_DIAGNOSTIC_NONFATAL_ERROR_TYPE: pxr.Tf.DiagnosticType # value = Tf.TF_DIAGNOSTIC_NONFATAL_ERROR_TYPE
TF_DIAGNOSTIC_RUNTIME_ERROR_TYPE: pxr.Tf.DiagnosticType # value = Tf.TF_DIAGNOSTIC_RUNTIME_ERROR_TYPE
TF_DIAGNOSTIC_STATUS_TYPE: pxr.Tf.DiagnosticType # value = Tf.TF_DIAGNOSTIC_STATUS_TYPE
TF_DIAGNOSTIC_WARNING_TYPE: pxr.Tf.DiagnosticType # value = Tf.TF_DIAGNOSTIC_WARNING_TYPE
_Alpha: pxr.Tf._TestEnum # value = Tf._Alpha
_Bravo: pxr.Tf._TestEnum # value = Tf._Bravo
_Charlie: pxr.Tf._TestEnum # value = Tf._Charlie
_Delta: pxr.Tf._TestEnum # value = Tf._Delta
__MFB_FULL_PACKAGE_NAME = 'tf'
