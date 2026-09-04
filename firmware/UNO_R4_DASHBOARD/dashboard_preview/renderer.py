"""Composed dashboard renderer."""

from .core import RendererCore
from .variants.analysis import AnalysisVariants
from .variants.baseline import BaselineVariant
from .variants.flow import FlowVariants
from .variants.gauges import GaugeVariants
from .variants.mode_pages import ModePageVariants


class DashboardRenderer(
    ModePageVariants,
    AnalysisVariants,
    GaugeVariants,
    FlowVariants,
    BaselineVariant,
    RendererCore,
):
    pass
