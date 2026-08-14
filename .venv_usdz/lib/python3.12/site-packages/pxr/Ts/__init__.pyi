from __future__ import annotations
import pxr.Ts._ts
import typing
import Boost.Python
import pxr.Tf
import pxr.Ts

__all__ = [
    "AntiRegressionAuthoringSelector",
    "AntiRegressionContain",
    "AntiRegressionKeepRatio",
    "AntiRegressionKeepStart",
    "AntiRegressionMode",
    "AntiRegressionNone",
    "ConvertFromStandardTangent",
    "ConvertToStandardTangent",
    "CurveType",
    "CurveTypeBezier",
    "CurveTypeHermite",
    "EditBehaviorBlock",
    "ExtrapHeld",
    "ExtrapLinear",
    "ExtrapLoopOscillate",
    "ExtrapLoopRepeat",
    "ExtrapLoopReset",
    "ExtrapMode",
    "ExtrapSloped",
    "ExtrapValueBlock",
    "Extrapolation",
    "InterpCurve",
    "InterpHeld",
    "InterpLinear",
    "InterpMode",
    "InterpValueBlock",
    "Knot",
    "KnotMap",
    "LoopParams",
    "RegressionPreventer",
    "SourceInnerLoopPostEcho",
    "SourceInnerLoopPreEcho",
    "SourceInnerLoopProto",
    "SourceKnotInterp",
    "SourcePostExtrap",
    "SourcePostExtrapLoop",
    "SourcePreExtrap",
    "SourcePreExtrapLoop",
    "Spline",
    "SplineSampleSource",
    "SplineSamples",
    "SplineSamplesWithSources",
    "TsTest_Museum",
    "TsTest_Sample",
    "TsTest_SampleBezier",
    "TsTest_SampleTimes",
    "TsTest_SplineData",
    "TsTest_TsEvaluator"
]


class AntiRegressionAuthoringSelector(Boost.Python.instance):
    pass
class AntiRegressionMode(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Ts.AntiRegressionNone, Ts.AntiRegressionContain, Ts.AntiRegressionKeepRatio, Ts.AntiRegressionKeepStart)
    pass
class CurveType(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Ts.CurveTypeBezier, Ts.CurveTypeHermite)
    pass
class EditBehaviorBlock(Boost.Python.instance):
    __instance_size__ = 32
    pass
class ExtrapMode(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Ts.ExtrapValueBlock, Ts.ExtrapHeld, Ts.ExtrapLinear, Ts.ExtrapSloped, Ts.ExtrapLoopRepeat, Ts.ExtrapLoopReset, Ts.ExtrapLoopOscillate)
    pass
class Extrapolation(Boost.Python.instance):
    @staticmethod
    def IsLooping(*args, **kwargs) -> None: ...
    @property
    def mode(self) -> None:
        """
        :type: None
        """
    @property
    def slope(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class InterpMode(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Ts.InterpValueBlock, Ts.InterpHeld, Ts.InterpLinear, Ts.InterpCurve)
    pass
class Knot(Boost.Python.instance):
    @staticmethod
    def ClearPreValue(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCurveType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCustomData(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCustomDataByKey(*args, **kwargs) -> None: ...
    @staticmethod
    def GetNextInterpolation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPostTanSlope(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPostTanWidth(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPreTanSlope(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPreTanWidth(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPreValue(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTime(*args, **kwargs) -> None: ...
    @staticmethod
    def GetValue(*args, **kwargs) -> None: ...
    @staticmethod
    def GetValueTypeName(*args, **kwargs) -> None: ...
    @staticmethod
    def IsDualValued(*args, **kwargs) -> None: ...
    @staticmethod
    def SetCurveType(*args, **kwargs) -> None: ...
    @staticmethod
    def SetCustomData(*args, **kwargs) -> None: ...
    @staticmethod
    def SetCustomDataByKey(*args, **kwargs) -> None: ...
    @staticmethod
    def SetNextInterpolation(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPostTanSlope(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPostTanWidth(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPreTanSlope(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPreTanWidth(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPreValue(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTime(*args, **kwargs) -> None: ...
    @staticmethod
    def SetValue(*args, **kwargs) -> None: ...
    pass
class KnotMap(Boost.Python.instance):
    @staticmethod
    def FindClosest(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTimeSpan(*args, **kwargs) -> None: ...
    @staticmethod
    def GetValueType(*args, **kwargs) -> None: ...
    @staticmethod
    def HasCurveSegments(*args, **kwargs) -> None: ...
    @staticmethod
    def clear(*args, **kwargs) -> None: ...
    @staticmethod
    def keys(*args, **kwargs) -> None: ...
    @staticmethod
    def values(*args, **kwargs) -> None: ...
    pass
class LoopParams(Boost.Python.instance):
    @staticmethod
    def GetLoopedInterval(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPrototypeInterval(*args, **kwargs) -> None: ...
    @property
    def numPostLoops(self) -> None:
        """
        :type: None
        """
    @property
    def numPreLoops(self) -> None:
        """
        :type: None
        """
    @property
    def protoEnd(self) -> None:
        """
        :type: None
        """
    @property
    def protoStart(self) -> None:
        """
        :type: None
        """
    @property
    def valueOffset(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 56
    pass
class RegressionPreventer(Boost.Python.instance):
    class InteractiveMode(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'RegressionPreventer'
        allValues: tuple # value = (Ts.RegressionPreventer.ModeLimitActive, Ts.RegressionPreventer.ModeLimitOpposite)
        pass
    class SetResult(Boost.Python.instance):
        @staticmethod
        def GetDebugDescription(*args, **kwargs) -> None: ...
        @property
        def adjusted(self) -> None:
            """
            :type: None
            """
        @property
        def havePostSegment(self) -> None:
            """
            :type: None
            """
        @property
        def havePreSegment(self) -> None:
            """
            :type: None
            """
        @property
        def postActiveAdjusted(self) -> None:
            """
            :type: None
            """
        @property
        def postActiveAdjustedWidth(self) -> None:
            """
            :type: None
            """
        @property
        def postOppositeAdjusted(self) -> None:
            """
            :type: None
            """
        @property
        def postOppositeAdjustedWidth(self) -> None:
            """
            :type: None
            """
        @property
        def preActiveAdjusted(self) -> None:
            """
            :type: None
            """
        @property
        def preActiveAdjustedWidth(self) -> None:
            """
            :type: None
            """
        @property
        def preOppositeAdjusted(self) -> None:
            """
            :type: None
            """
        @property
        def preOppositeAdjustedWidth(self) -> None:
            """
            :type: None
            """
        pass
    @staticmethod
    def Set(*args, **kwargs) -> None: ...
    ModeLimitActive: pxr.Ts.InteractiveMode # value = Ts.RegressionPreventer.ModeLimitActive
    ModeLimitOpposite: pxr.Ts.InteractiveMode # value = Ts.RegressionPreventer.ModeLimitOpposite
    pass
class Spline(Boost.Python.instance):
    @staticmethod
    def AdjustRegressiveTangents(*args, **kwargs) -> None: ...
    @staticmethod
    def ClearKnots(*args, **kwargs) -> None: ...
    @staticmethod
    def DoSidesDiffer(*args, **kwargs) -> None: ...
    @staticmethod
    def Eval(*args, **kwargs) -> None: ...
    @staticmethod
    def EvalDerivative(*args, **kwargs) -> None: ...
    @staticmethod
    def EvalHeld(*args, **kwargs) -> None: ...
    @staticmethod
    def EvalPreDerivative(*args, **kwargs) -> None: ...
    @staticmethod
    def EvalPreValue(*args, **kwargs) -> None: ...
    @staticmethod
    def EvalPreValueHeld(*args, **kwargs) -> None: ...
    @staticmethod
    def GetAntiRegressionAuthoringMode(*args, **kwargs) -> None: ...
    @staticmethod
    def GetCurveType(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInnerLoopParams(*args, **kwargs) -> None: ...
    @staticmethod
    def GetKnot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetKnots(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPostExtrapolation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPreExtrapolation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetValueTypeName(*args, **kwargs) -> None: ...
    @staticmethod
    def HasExtrapolatingLoops(*args, **kwargs) -> None: ...
    @staticmethod
    def HasInnerLoops(*args, **kwargs) -> None: ...
    @staticmethod
    def HasLoops(*args, **kwargs) -> None: ...
    @staticmethod
    def HasRegressiveTangents(*args, **kwargs) -> None: ...
    @staticmethod
    def HasValueBlockAtTime(*args, **kwargs) -> None: ...
    @staticmethod
    def HasValueBlocks(*args, **kwargs) -> None: ...
    @staticmethod
    def IsEmpty(*args, **kwargs) -> None: ...
    @staticmethod
    def IsTimeValued(*args, **kwargs) -> None: ...
    @staticmethod
    def RemoveKnot(*args, **kwargs) -> None: ...
    @staticmethod
    def Sample(*args, **kwargs) -> None: ...
    @staticmethod
    def SetCurveType(*args, **kwargs) -> None: ...
    @staticmethod
    def SetInnerLoopParams(*args, **kwargs) -> None: ...
    @staticmethod
    def SetKnot(*args, **kwargs) -> None: ...
    @staticmethod
    def SetKnots(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPostExtrapolation(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPreExtrapolation(*args, **kwargs) -> None: ...
    @staticmethod
    def SetTimeValued(*args, **kwargs) -> None: ...
    pass
class SplineSampleSource(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
    @staticmethod
    def GetValueFromName(*args, **kwargs) -> None: ...
    _baseName = ''
    allValues: tuple # value = (Ts.SourcePreExtrap, Ts.SourcePreExtrapLoop, Ts.SourceInnerLoopPreEcho, Ts.SourceInnerLoopProto, Ts.SourceInnerLoopPostEcho, Ts.SourceKnotInterp, Ts.SourcePostExtrap, Ts.SourcePostExtrapLoop)
    pass
class SplineSamples(Boost.Python.instance):
    @property
    def polylines(self) -> None:
        """
        :type: None
        """
    pass
class SplineSamplesWithSources(Boost.Python.instance):
    @property
    def polylines(self) -> None:
        """
        :type: None
        """
    @property
    def sources(self) -> None:
        """
        :type: None
        """
    pass
class TsTest_Museum(Boost.Python.instance):
    class DataId(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'Test_Museum'
        allValues: tuple # value = (Ts.Test_Museum.TwoKnotBezier, Ts.Test_Museum.TwoKnotLinear, Ts.Test_Museum.FourKnotBezier, Ts.Test_Museum.SimpleInnerLoop, Ts.Test_Museum.InnerLoop2and2, Ts.Test_Museum.InnerLoopPre, Ts.Test_Museum.InnerLoopPost, Ts.Test_Museum.ExtrapLoopRepeat, Ts.Test_Museum.ExtrapLoopReset, Ts.Test_Museum.ExtrapLoopOscillate, Ts.Test_Museum.InnerAndExtrapLoops, Ts.Test_Museum.RegressiveLoop, Ts.Test_Museum.RegressiveS, Ts.Test_Museum.RegressiveSStandard, Ts.Test_Museum.RegressiveSPreOut, Ts.Test_Museum.RegressiveSPostOut, Ts.Test_Museum.RegressiveSBothOut, Ts.Test_Museum.RegressivePreJ, Ts.Test_Museum.RegressivePostJ, Ts.Test_Museum.RegressivePreC, Ts.Test_Museum.RegressivePostC, Ts.Test_Museum.RegressivePreG, Ts.Test_Museum.RegressivePostG, Ts.Test_Museum.RegressivePreFringe, Ts.Test_Museum.RegressivePostFringe, Ts.Test_Museum.BoldS, Ts.Test_Museum.Cusp, Ts.Test_Museum.CenterVertical, Ts.Test_Museum.NearCenterVertical, Ts.Test_Museum.VerticalTorture, Ts.Test_Museum.FourThirdOneThird, Ts.Test_Museum.OneThirdFourThird, Ts.Test_Museum.StartVert, Ts.Test_Museum.EndVert, Ts.Test_Museum.FringeVert, Ts.Test_Museum.MarginalN, Ts.Test_Museum.ZeroTans, Ts.Test_Museum.ComplexParams)
        pass
    @staticmethod
    def GetAllNames(*args, **kwargs) -> None: ...
    @staticmethod
    def GetData(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDataByName(*args, **kwargs) -> None: ...
    BoldS: pxr.Ts.DataId # value = Ts.Test_Museum.BoldS
    CenterVertical: pxr.Ts.DataId # value = Ts.Test_Museum.CenterVertical
    ComplexParams: pxr.Ts.DataId # value = Ts.Test_Museum.ComplexParams
    Cusp: pxr.Ts.DataId # value = Ts.Test_Museum.Cusp
    EndVert: pxr.Ts.DataId # value = Ts.Test_Museum.EndVert
    ExtrapLoopOscillate: pxr.Ts.DataId # value = Ts.Test_Museum.ExtrapLoopOscillate
    ExtrapLoopRepeat: pxr.Ts.DataId # value = Ts.Test_Museum.ExtrapLoopRepeat
    ExtrapLoopReset: pxr.Ts.DataId # value = Ts.Test_Museum.ExtrapLoopReset
    FourKnotBezier: pxr.Ts.DataId # value = Ts.Test_Museum.FourKnotBezier
    FourThirdOneThird: pxr.Ts.DataId # value = Ts.Test_Museum.FourThirdOneThird
    FringeVert: pxr.Ts.DataId # value = Ts.Test_Museum.FringeVert
    InnerAndExtrapLoops: pxr.Ts.DataId # value = Ts.Test_Museum.InnerAndExtrapLoops
    InnerLoop2and2: pxr.Ts.DataId # value = Ts.Test_Museum.InnerLoop2and2
    InnerLoopPost: pxr.Ts.DataId # value = Ts.Test_Museum.InnerLoopPost
    InnerLoopPre: pxr.Ts.DataId # value = Ts.Test_Museum.InnerLoopPre
    MarginalN: pxr.Ts.DataId # value = Ts.Test_Museum.MarginalN
    NearCenterVertical: pxr.Ts.DataId # value = Ts.Test_Museum.NearCenterVertical
    OneThirdFourThird: pxr.Ts.DataId # value = Ts.Test_Museum.OneThirdFourThird
    RegressiveLoop: pxr.Ts.DataId # value = Ts.Test_Museum.RegressiveLoop
    RegressivePostC: pxr.Ts.DataId # value = Ts.Test_Museum.RegressivePostC
    RegressivePostFringe: pxr.Ts.DataId # value = Ts.Test_Museum.RegressivePostFringe
    RegressivePostG: pxr.Ts.DataId # value = Ts.Test_Museum.RegressivePostG
    RegressivePostJ: pxr.Ts.DataId # value = Ts.Test_Museum.RegressivePostJ
    RegressivePreC: pxr.Ts.DataId # value = Ts.Test_Museum.RegressivePreC
    RegressivePreFringe: pxr.Ts.DataId # value = Ts.Test_Museum.RegressivePreFringe
    RegressivePreG: pxr.Ts.DataId # value = Ts.Test_Museum.RegressivePreG
    RegressivePreJ: pxr.Ts.DataId # value = Ts.Test_Museum.RegressivePreJ
    RegressiveS: pxr.Ts.DataId # value = Ts.Test_Museum.RegressiveS
    RegressiveSBothOut: pxr.Ts.DataId # value = Ts.Test_Museum.RegressiveSBothOut
    RegressiveSPostOut: pxr.Ts.DataId # value = Ts.Test_Museum.RegressiveSPostOut
    RegressiveSPreOut: pxr.Ts.DataId # value = Ts.Test_Museum.RegressiveSPreOut
    RegressiveSStandard: pxr.Ts.DataId # value = Ts.Test_Museum.RegressiveSStandard
    SimpleInnerLoop: pxr.Ts.DataId # value = Ts.Test_Museum.SimpleInnerLoop
    StartVert: pxr.Ts.DataId # value = Ts.Test_Museum.StartVert
    TwoKnotBezier: pxr.Ts.DataId # value = Ts.Test_Museum.TwoKnotBezier
    TwoKnotLinear: pxr.Ts.DataId # value = Ts.Test_Museum.TwoKnotLinear
    VerticalTorture: pxr.Ts.DataId # value = Ts.Test_Museum.VerticalTorture
    ZeroTans: pxr.Ts.DataId # value = Ts.Test_Museum.ZeroTans
    pass
class TsTest_Sample(Boost.Python.instance):
    @property
    def time(self) -> None:
        """
        :type: None
        """
    @property
    def value(self) -> None:
        """
        :type: None
        """
    __instance_size__ = 40
    pass
class TsTest_SampleTimes(Boost.Python.instance):
    class SampleTime(Boost.Python.instance):
        @property
        def pre(self) -> None:
            """
            :type: None
            """
        @property
        def time(self) -> None:
            """
            :type: None
            """
        __instance_size__ = 40
        pass
    @staticmethod
    def AddExtrapolatingLoopTimes(*args, **kwargs) -> None: ...
    @staticmethod
    def AddExtrapolationTimes(*args, **kwargs) -> None: ...
    @staticmethod
    def AddKnotTimes(*args, **kwargs) -> None: ...
    @staticmethod
    def AddStandardTimes(*args, **kwargs) -> None: ...
    @staticmethod
    def AddTimes(*args, **kwargs) -> None: ...
    @staticmethod
    def AddUniformInterpolationTimes(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMaxTime(*args, **kwargs) -> None: ...
    @staticmethod
    def GetMinTime(*args, **kwargs) -> None: ...
    @staticmethod
    def GetTimes(*args, **kwargs) -> None: ...
    pass
class TsTest_SplineData(Boost.Python.instance):
    class ExtrapMethod(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'Test_SplineData'
        allValues: tuple # value = (Ts.Test_SplineData.ExtrapHeld, Ts.Test_SplineData.ExtrapLinear, Ts.Test_SplineData.ExtrapSloped, Ts.Test_SplineData.ExtrapLoop)
        pass
    class Extrapolation(Boost.Python.instance):
        @property
        def loopMode(self) -> None:
            """
            :type: None
            """
        @property
        def method(self) -> None:
            """
            :type: None
            """
        @property
        def slope(self) -> None:
            """
            :type: None
            """
        pass
    class Feature(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'Test_SplineData'
        allValues: tuple # value = (Ts.Test_SplineData.FeatureHeldSegments, Ts.Test_SplineData.FeatureLinearSegments, Ts.Test_SplineData.FeatureBezierSegments, Ts.Test_SplineData.FeatureHermiteSegments, Ts.Test_SplineData.FeatureDualValuedKnots, Ts.Test_SplineData.FeatureInnerLoops, Ts.Test_SplineData.FeatureExtrapolatingLoops)
        pass
    class InnerLoopParams(Boost.Python.instance):
        @staticmethod
        def IsValid(*args, **kwargs) -> None: ...
        @property
        def enabled(self) -> None:
            """
            :type: None
            """
        @property
        def numPostLoops(self) -> None:
            """
            :type: None
            """
        @property
        def numPreLoops(self) -> None:
            """
            :type: None
            """
        @property
        def protoEnd(self) -> None:
            """
            :type: None
            """
        @property
        def protoStart(self) -> None:
            """
            :type: None
            """
        @property
        def valueOffset(self) -> None:
            """
            :type: None
            """
        pass
    class InterpMethod(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'Test_SplineData'
        allValues: tuple # value = (Ts.Test_SplineData.InterpHeld, Ts.Test_SplineData.InterpLinear, Ts.Test_SplineData.InterpCurve)
        pass
    class Knot(Boost.Python.instance):
        @property
        def isDualValued(self) -> None:
            """
            :type: None
            """
        @property
        def nextSegInterpMethod(self) -> None:
            """
            :type: None
            """
        @property
        def postAuto(self) -> None:
            """
            :type: None
            """
        @property
        def postLen(self) -> None:
            """
            :type: None
            """
        @property
        def postSlope(self) -> None:
            """
            :type: None
            """
        @property
        def preAuto(self) -> None:
            """
            :type: None
            """
        @property
        def preLen(self) -> None:
            """
            :type: None
            """
        @property
        def preSlope(self) -> None:
            """
            :type: None
            """
        @property
        def preValue(self) -> None:
            """
            :type: None
            """
        @property
        def time(self) -> None:
            """
            :type: None
            """
        @property
        def value(self) -> None:
            """
            :type: None
            """
        pass
    class LoopMode(pxr.Tf.Tf_PyEnumWrapper, pxr.Tf.Enum, Boost.Python.instance):
        @staticmethod
        def GetValueFromName(*args, **kwargs) -> None: ...
        _baseName = 'Test_SplineData'
        allValues: tuple # value = (Ts.Test_SplineData.LoopNone, Ts.Test_SplineData.LoopContinue, Ts.Test_SplineData.LoopRepeat, Ts.Test_SplineData.LoopReset, Ts.Test_SplineData.LoopOscillate)
        pass
    @staticmethod
    def AddKnot(*args, **kwargs) -> None: ...
    @staticmethod
    def GetDebugDescription(*args, **kwargs) -> None: ...
    @staticmethod
    def GetInnerLoopParams(*args, **kwargs) -> None: ...
    @staticmethod
    def GetIsHermite(*args, **kwargs) -> None: ...
    @staticmethod
    def GetKnots(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPostExtrapolation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetPreExtrapolation(*args, **kwargs) -> None: ...
    @staticmethod
    def GetRequiredFeatures(*args, **kwargs) -> None: ...
    @staticmethod
    def SetInnerLoopParams(*args, **kwargs) -> None: ...
    @staticmethod
    def SetIsHermite(*args, **kwargs) -> None: ...
    @staticmethod
    def SetKnots(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPostExtrapolation(*args, **kwargs) -> None: ...
    @staticmethod
    def SetPreExtrapolation(*args, **kwargs) -> None: ...
    ExtrapHeld: pxr.Ts.ExtrapMethod # value = Ts.Test_SplineData.ExtrapHeld
    ExtrapLinear: pxr.Ts.ExtrapMethod # value = Ts.Test_SplineData.ExtrapLinear
    ExtrapLoop: pxr.Ts.ExtrapMethod # value = Ts.Test_SplineData.ExtrapLoop
    ExtrapSloped: pxr.Ts.ExtrapMethod # value = Ts.Test_SplineData.ExtrapSloped
    FeatureBezierSegments: pxr.Ts.Feature # value = Ts.Test_SplineData.FeatureBezierSegments
    FeatureDualValuedKnots: pxr.Ts.Feature # value = Ts.Test_SplineData.FeatureDualValuedKnots
    FeatureExtrapolatingLoops: pxr.Ts.Feature # value = Ts.Test_SplineData.FeatureExtrapolatingLoops
    FeatureHeldSegments: pxr.Ts.Feature # value = Ts.Test_SplineData.FeatureHeldSegments
    FeatureHermiteSegments: pxr.Ts.Feature # value = Ts.Test_SplineData.FeatureHermiteSegments
    FeatureInnerLoops: pxr.Ts.Feature # value = Ts.Test_SplineData.FeatureInnerLoops
    FeatureLinearSegments: pxr.Ts.Feature # value = Ts.Test_SplineData.FeatureLinearSegments
    InterpCurve: pxr.Ts.InterpMethod # value = Ts.Test_SplineData.InterpCurve
    InterpHeld: pxr.Ts.InterpMethod # value = Ts.Test_SplineData.InterpHeld
    InterpLinear: pxr.Ts.InterpMethod # value = Ts.Test_SplineData.InterpLinear
    LoopContinue: pxr.Ts.LoopMode # value = Ts.Test_SplineData.LoopContinue
    LoopNone: pxr.Ts.LoopMode # value = Ts.Test_SplineData.LoopNone
    LoopOscillate: pxr.Ts.LoopMode # value = Ts.Test_SplineData.LoopOscillate
    LoopRepeat: pxr.Ts.LoopMode # value = Ts.Test_SplineData.LoopRepeat
    LoopReset: pxr.Ts.LoopMode # value = Ts.Test_SplineData.LoopReset
    pass
class TsTest_TsEvaluator(Boost.Python.instance):
    @staticmethod
    def Eval(*args, **kwargs) -> None: ...
    @staticmethod
    def Sample(*args, **kwargs) -> None: ...
    @staticmethod
    def SplineDataToSpline(*args, **kwargs) -> None: ...
    @staticmethod
    def SplineToSplineData(*args, **kwargs) -> None: ...
    __instance_size__ = 32
    pass
def ConvertFromStandardTangent(*args, **kwargs) -> None:
    pass
def ConvertToStandardTangent(*args, **kwargs) -> None:
    pass
def TsTest_SampleBezier(*args, **kwargs) -> None:
    pass
AntiRegressionContain: pxr.Ts.AntiRegressionMode # value = Ts.AntiRegressionContain
AntiRegressionKeepRatio: pxr.Ts.AntiRegressionMode # value = Ts.AntiRegressionKeepRatio
AntiRegressionKeepStart: pxr.Ts.AntiRegressionMode # value = Ts.AntiRegressionKeepStart
AntiRegressionNone: pxr.Ts.AntiRegressionMode # value = Ts.AntiRegressionNone
CurveTypeBezier: pxr.Ts.CurveType # value = Ts.CurveTypeBezier
CurveTypeHermite: pxr.Ts.CurveType # value = Ts.CurveTypeHermite
ExtrapHeld: pxr.Ts.ExtrapMode # value = Ts.ExtrapHeld
ExtrapLinear: pxr.Ts.ExtrapMode # value = Ts.ExtrapLinear
ExtrapLoopOscillate: pxr.Ts.ExtrapMode # value = Ts.ExtrapLoopOscillate
ExtrapLoopRepeat: pxr.Ts.ExtrapMode # value = Ts.ExtrapLoopRepeat
ExtrapLoopReset: pxr.Ts.ExtrapMode # value = Ts.ExtrapLoopReset
ExtrapSloped: pxr.Ts.ExtrapMode # value = Ts.ExtrapSloped
ExtrapValueBlock: pxr.Ts.ExtrapMode # value = Ts.ExtrapValueBlock
InterpCurve: pxr.Ts.InterpMode # value = Ts.InterpCurve
InterpHeld: pxr.Ts.InterpMode # value = Ts.InterpHeld
InterpLinear: pxr.Ts.InterpMode # value = Ts.InterpLinear
InterpValueBlock: pxr.Ts.InterpMode # value = Ts.InterpValueBlock
SourceInnerLoopPostEcho: pxr.Ts.SplineSampleSource # value = Ts.SourceInnerLoopPostEcho
SourceInnerLoopPreEcho: pxr.Ts.SplineSampleSource # value = Ts.SourceInnerLoopPreEcho
SourceInnerLoopProto: pxr.Ts.SplineSampleSource # value = Ts.SourceInnerLoopProto
SourceKnotInterp: pxr.Ts.SplineSampleSource # value = Ts.SourceKnotInterp
SourcePostExtrap: pxr.Ts.SplineSampleSource # value = Ts.SourcePostExtrap
SourcePostExtrapLoop: pxr.Ts.SplineSampleSource # value = Ts.SourcePostExtrapLoop
SourcePreExtrap: pxr.Ts.SplineSampleSource # value = Ts.SourcePreExtrap
SourcePreExtrapLoop: pxr.Ts.SplineSampleSource # value = Ts.SourcePreExtrapLoop
__MFB_FULL_PACKAGE_NAME = 'ts'
