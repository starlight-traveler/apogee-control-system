#ifndef MATPLOT_DETAIL_EXPORTS_H
#define MATPLOT_DETAIL_EXPORTS_H

#if defined(_WIN32) && !defined(MATPLOT_STATIC)
#    if defined(MATPLOT_EXPORTING)
#        define MATPLOT_EXPORTS __declspec(dllexport)
#    else
#        define MATPLOT_EXPORTS __declspec(dllimport)
#    endif
#else
#    define MATPLOT_EXPORTS
#endif

#endif  // MATPLOT_DETAIL_EXPORTS_H