/*
 * simlink-lib.c - simlink-Bibliothek als eigene Uebersetzungseinheit
 * (ohne vice.h, damit windows.h bzw. POSIX-Header nicht mit VICE-Typen kollidieren).
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 */
#if !defined(__MSDOS__)
#include "../simlink/simlink.c"
#else
typedef int parallel_simlink_lib_unused;
#endif
