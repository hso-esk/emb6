#******************************************************************************
#
# Makefile - Rules for building the SRF06EB BSP documentation for CC26xx.
#
# Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
# TI Information - Selective Disclosure
#
#******************************************************************************

#
# Skip the definitions of the following variables if we are only
# attempting to clean the folder up.  These variables are not needed
# for the "clean" and "spellclean" targets.
#
ifneq (${MAKECMDGOALS},clean)
ifneq (${MAKECMDGOALS},spellclean)

#
# Device name
#
CCDEVICE:=CC26xx

#
# Board name
#
CCBOARD:=SmartRF06EB

#
# Board/device folder
#
CCFBOARD:=srf06eb
CCFDEVICE:=cc26xx
CCFOLDER:=${CCFBOARD}\\\\_${CCFDEVICE}

#
# Board specifics
#
BOARDKEYCOUNT:=5
BOARDLEDCOUNT:=4

#
# The title of this document.
#
TITLE:=${CCBOARD} Board Support Package
SUBTITLE:=Texas Instruments ${CCDEVICE} Family of Products
TITLE2:=User's Guide

#
# Title in document properties 
#
DOCPROPERTIESTITLE:=${TITLE} ${TITLE2}, ${SUBTITLE}

#
# The document number for this document.
#
DOCNUM:=SWRU352

#
# The document revision, i.e. <<blank>> (first revision), A, B etc.
#
DOCREV:=-prelim-0-2

#
# Document watermark
#
# No watermark: "\\\\ " (without ")
DOCWATERMARK:=PRELIMINARY

#
# The document disclosure settings
#
#DISCLOSURE1:=TI Confidential
#DISCLOSURE2:=NDA Restrictions
DISCLOSURE1:=CC BY-ND 3.0
DISCLOSURE2:=Texas Instruments Incorporated

#
# Get the repository revision number.
#
# Working copy version (delete revision ranges and modifiers)
#VERSION:=`svnversion ../source | sed 's/.*://g' | sed 's/[M|S|P]//g'`
# Repository HEAD revision
SVNREV:=${shell svn info http://webix.norway.design.ti.com/svn/satsw | \
          grep "Revision" | cut -d\  -f2}

#
# Get the modification date of the repository.
#
# Using today's date
DATE:=${shell date "+%B %d, %G"}
YEAR:=${shell date "+%Y"}

endif	#ifneq (${MAKECMDGOALS},spellclean)
endif	#ifneq (${MAKECMDGOALS},clean)

#
# Here you can specify which doxygen version to use if you have multiple installed
# E.g:
DOXYGEN:=/cygdrive/c/Program\ Files\ \(x86\)/doxygen/doxygen_1_4_6/bin/doxygen
#DOXYGEN:=doxygen

#
# Document Name
#
DOCNAME:=${DOCNUM}${DOCREV}

#
# The default rule, which causes the document to be built.
#
all: ${DOCNAME}.pdf

#
# Rule to run spell checking
#
spell: spell-check.txt

#
# The rule to clean out all the build products.
#
clean: spellclean
	@rm -rf ${wildcard *.pdf} latex xml ${wildcard *~}
	@rm -rf ${wildcard code_*.dox} board_chapters.dox

spellclean:
	@rm -f spell-check.txt combined-tex combined-dict.rws
	@rm -f latex.txt suspicious.txt

#
# The rule for building the document.
#
${DOCNAME}.pdf: Doxyfile
${DOCNAME}.pdf: Makefile
${DOCNAME}.pdf: ${wildcard *.dox}
${DOCNAME}.pdf: ${wildcard ../source/*.c}
${DOCNAME}.pdf: ${wildcard ../source/*.h}
${DOCNAME}.pdf: ${wildcard ../../../doc/custom_files/*}
	@set -e;                                                                \
	 echo "  doxygen";                                                      \
	 ${DOXYGEN};                                                            \
	 sed -i -e "s/\\\\-//g" latex/*.tex;                                    \
	 sed -i -e "s/\\\begin{Code}//g" latex/*.tex;                           \
	 sed -i -e "s/\\\end{Code}//g" latex/*.tex;                             \
	 cp ../../../doc/custom_files/* latex;                                  \
	 sed -i -e "s/CCFOLDER/${CCFOLDER}/g" latex/*.tex;                      \
	 sed -i -e "s/CCFBOARD/${CCFBOARD}/g" latex/*.tex;                      \
	 sed -i -e "s/CCFDEVICE/${CCFDEVICE}/g" latex/*.tex;                    \
	 sed -i -e "s/CCDEVICE/${CCDEVICE}/g" latex/*.tex;                      \
	 sed -i -e "s/CCBOARD/${CCBOARD}/g" latex/*.tex;                        \
	 sed -i -e "s/BOARDLEDCOUNT/${BOARDLEDCOUNT}/g" latex/*.tex;            \
	 sed -i -e "s/BOARDKEYCOUNT/${BOARDKEYCOUNT}/g" latex/*.tex;            \
	 TITLE=`echo ${TITLE} | sed 's/{\\\\textregistered}//g'`;               \
	 sed -i "s/DOCTITLE/${DOCPROPERTIESTITLE}/g" latex/refman.tex;          \
	 sed -i "s/COVERTITLE1/${TITLE}/g" latex/refman.tex;                    \
	 sed -i "s/COVERTITLESUB1/${SUBTITLE}/g" latex/refman.tex;              \
	 sed -i "s/COVERTITLE2/${TITLE2}/g" latex/refman.tex;                   \
	 sed -i "s/DISCLOSURE1/${DISCLOSURE1}/g" latex/refman.tex;              \
	 sed -i "s/DISCLOSURE2/${DISCLOSURE2}/g" latex/refman.tex;              \
	 sed -i "s/DOCNUM/${DOCNAME}/g" latex/refman.tex;                       \
	 sed -i "s/PRELIMINARY/${DOCWATERMARK}/g" latex/refman.tex;             \
	 sed -i "s/__REVISION__/${SVNREV}/g" latex/refman.tex;                  \
	 sed -i -e "s/YEAR/${YEAR}/g" latex/*.tex;                              \
	 sed -i "s/COPYRIGHT/${YEAR}/g" latex/refman.tex;                       \
	 sed -i "s/DATE/${DATE}/g" latex/refman.tex;                            \
	 perl -w -i.orig ../../../doc/scripts/convert.pl latex/group__*.tex;    \
	 sed -i "s/,/, /g" latex/group__*.tex;                                  \
	 perl -w -i.orig ../../../doc/scripts/verbatim.pl latex/*.tex;          \
	 rm -f latex/*.tex.orig;                                                \
	 rm -f latex/refman.pdf;                                                \
	 echo "  pdflatex ${@:.pdf=.tex}";                                      \
	 make -C latex refman.pdf;                                              \
	 cp latex/refman.pdf ${DOCNAME}.pdf;                                    \
	 grep -i warning latex/refman.log || true

#
# Rule for creating the spell check files
#
# The output file spell-check.txt will contain a list of suspicious words it
# found in the documentation.  If a word in the list is okay, you can add it to
# words.wl.
#
spell-check.txt: ${DOCNAME}.pdf
	@echo "  spell checking with aspell (if aspell is installed)"
	@echo "    see file spell-check.txt for suspicious words"
	@aspell --lang=en --variety=US create master ./combined-dict.rws < words.wl
	@grep -hv ^% ${wildcard latex/*.tex} > ./combined-tex
	@aspell -t --add-extra-dicts=./combined-dict.rws list <combined-tex \
	     | sort | uniq >spell-check.txt
	@rm -f ./combined-tex ./combined-dict.rws
	@grep -E ^[a-g]*$$ spell-check.txt > latex.txt || true
	@grep -vE ^[a-g]*$$ spell-check.txt > suspicious.txt || true

foobar:
	@sed -n '/\\code/,/\\endcode/p' interrupt.dox | grep -v ^\\\\.\*code$
