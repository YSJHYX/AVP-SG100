from __future__ import annotations
import pxr.Trace._trace
import typing
import Boost.Python
import pxr.Trace

__all__ = [
    "AggregateNode",
    "AggregateTree",
    "Collector",
    "GetElapsedSeconds",
    "GetTestEventName",
    "PythonGarbageCollectionCallback",
    "Reporter",
    "TestAuto",
    "TestCreateEvents",
    "TestNesting"
]


class AggregateNode(Boost.Python.instance):
    @staticmethod
    def Append(*args, **kwargs) -> None: ...
    @property
    def children(self) -> None:
        """
        :type: None
        """
    @property
    def count(self) -> None:
        """
        :type: None
        """
    @property
    def exclusiveCount(self) -> None:
        """
        :type: None
        """
    @property
    def exclusiveTime(self) -> None:
        """
        :type: None
        """
    @property
    def expanded(self) -> None:
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
    def id(self) -> None:
        """
        :type: None
        """
    @property
    def inclusiveTime(self) -> None:
        """
        :type: None
        """
    @property
    def key(self) -> None:
        """
        :type: None
        """
    pass
class AggregateTree(Boost.Python.instance):
    @property
    def expired(self) -> None:
        """
        True if this object has expired, False otherwise.

        :type: None
        """
    @property
    def root(self) -> None:
        """
        :type: None
        """
    pass
class Collector(Boost.Python.instance):
    @staticmethod
    def BeginEvent(*args, **kwargs) -> None: ...
    @staticmethod
    def BeginEventAtTime(*args, **kwargs) -> None: ...
    @staticmethod
    def Clear(*args, **kwargs) -> None: ...
    @staticmethod
    def EndEvent(*args, **kwargs) -> None: ...
    @staticmethod
    def EndEventAtTime(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLabel(*args, **kwargs) -> None: ...
    @property
    def enabled(self) -> None:
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
    def pythonTracingEnabled(self) -> None:
        """
        :type: None
        """
    pass
class Reporter(Boost.Python.instance):
    class ParsedTree(Boost.Python.instance):
        @property
        def iterationCount(self) -> None:
            """
            :type: None
            """
        @property
        def tree(self) -> None:
            """
            :type: None
            """
        pass
    @staticmethod
    def ClearTree(*args, **kwargs) -> None: ...
    @staticmethod
    def GetLabel(*args, **kwargs) -> None: ...
    @staticmethod
    def LoadReport(*args, **kwargs) -> None: ...
    @staticmethod
    def Report(*args, **kwargs) -> None: ...
    @staticmethod
    def ReportChromeTracing(*args, **kwargs) -> None: ...
    @staticmethod
    def ReportChromeTracingToFile(*args, **kwargs) -> None: ...
    @staticmethod
    def ReportTimes(*args, **kwargs) -> None: ...
    @staticmethod
    def UpdateTraceTrees(*args, **kwargs) -> None: ...
    @property
    def aggregateTreeRoot(self) -> None:
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
    def foldRecursiveCalls(self) -> None:
        """
        :type: None
        """
    @property
    def groupByFunction(self) -> None:
        """
        :type: None
        """
    @property
    def shouldAdjustForOverheadAndNoise(self) -> None:
        """
        :type: None
        """
    globalReporter: pxr.Trace.Reporter
    pass
def GetElapsedSeconds(*args, **kwargs) -> None:
    pass
def GetTestEventName(*args, **kwargs) -> None:
    pass
def PythonGarbageCollectionCallback(*args, **kwargs) -> None:
    pass
def TestAuto(*args, **kwargs) -> None:
    pass
def TestCreateEvents(*args, **kwargs) -> None:
    pass
def TestNesting(*args, **kwargs) -> None:
    pass
__MFB_FULL_PACKAGE_NAME = 'trace'
