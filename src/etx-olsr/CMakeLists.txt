build_lib(
  LIBNAME etx-olsr
  SOURCE_FILES
    helper/control-agent-helper.cc
    helper/etx-olsr-helper.cc
    helper/etx-olsr-hopcount-helper.cc
    model/control-agent.cc
    model/etx-olsr-hopcount-routing-protocol.cc
    model/etx-olsr-msg-header.cc
    model/etx-olsr-routing-protocol.cc
    model/etx-olsr-state.cc
    model/etx-tc-metric-trailer.cc
  HEADER_FILES
    helper/control-agent-helper.h
    helper/etx-olsr-helper.h
    helper/etx-olsr-hopcount-helper.h
    model/control-agent.h
    model/etx-olsr-hopcount-routing-protocol.h
    model/etx-olsr-msg-header.h
    model/etx-olsr-repositories.h
    model/etx-olsr-routing-protocol.h
    model/etx-olsr-state.h
    model/etx-tc-metric-trailer.h
  LIBRARIES_TO_LINK ${libinternet}
)
