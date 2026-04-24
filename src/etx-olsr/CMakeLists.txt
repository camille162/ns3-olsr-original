build_lib(
  LIBNAME etx-olsr
  SOURCE_FILES
    helper/etx-olsr-helper.cc
    model/etx-olsr-routing-protocol.cc
  HEADER_FILES
    helper/etx-olsr-helper.h
    model/etx-olsr-routing-protocol.h
  LIBRARIES_TO_LINK
    ${libinternet}
    ${libolsr}
)
