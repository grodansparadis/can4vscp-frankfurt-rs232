#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-relocated.mk)" "nbproject/Makefile-local-relocated.mk"
include nbproject/Makefile-local-relocated.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=relocated
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=main.c ECAN.c fifo.c eeprom.c crc8.c hexutils.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.p1 ${OBJECTDIR}/ECAN.p1 ${OBJECTDIR}/fifo.p1 ${OBJECTDIR}/eeprom.p1 ${OBJECTDIR}/crc8.p1 ${OBJECTDIR}/hexutils.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/main.p1.d ${OBJECTDIR}/ECAN.p1.d ${OBJECTDIR}/fifo.p1.d ${OBJECTDIR}/eeprom.p1.d ${OBJECTDIR}/crc8.p1.d ${OBJECTDIR}/hexutils.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.p1 ${OBJECTDIR}/ECAN.p1 ${OBJECTDIR}/fifo.p1 ${OBJECTDIR}/eeprom.p1 ${OBJECTDIR}/crc8.p1 ${OBJECTDIR}/hexutils.p1

# Source Files
SOURCEFILES=main.c ECAN.c fifo.c eeprom.c crc8.c hexutils.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-relocated.mk dist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F2580
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=realice  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/main.p1  main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECAN.p1: ECAN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ECAN.p1.d 
	@${RM} ${OBJECTDIR}/ECAN.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=realice  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/ECAN.p1  ECAN.c 
	@-${MV} ${OBJECTDIR}/ECAN.d ${OBJECTDIR}/ECAN.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECAN.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/fifo.p1: fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/fifo.p1.d 
	@${RM} ${OBJECTDIR}/fifo.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=realice  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/fifo.p1  fifo.c 
	@-${MV} ${OBJECTDIR}/fifo.d ${OBJECTDIR}/fifo.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/fifo.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/eeprom.p1: eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eeprom.p1.d 
	@${RM} ${OBJECTDIR}/eeprom.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=realice  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/eeprom.p1  eeprom.c 
	@-${MV} ${OBJECTDIR}/eeprom.d ${OBJECTDIR}/eeprom.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/eeprom.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/crc8.p1: crc8.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/crc8.p1.d 
	@${RM} ${OBJECTDIR}/crc8.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=realice  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/crc8.p1  crc8.c 
	@-${MV} ${OBJECTDIR}/crc8.d ${OBJECTDIR}/crc8.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/crc8.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/hexutils.p1: hexutils.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/hexutils.p1.d 
	@${RM} ${OBJECTDIR}/hexutils.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=realice  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/hexutils.p1  hexutils.c 
	@-${MV} ${OBJECTDIR}/hexutils.d ${OBJECTDIR}/hexutils.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/hexutils.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/main.p1  main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECAN.p1: ECAN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ECAN.p1.d 
	@${RM} ${OBJECTDIR}/ECAN.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/ECAN.p1  ECAN.c 
	@-${MV} ${OBJECTDIR}/ECAN.d ${OBJECTDIR}/ECAN.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECAN.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/fifo.p1: fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/fifo.p1.d 
	@${RM} ${OBJECTDIR}/fifo.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/fifo.p1  fifo.c 
	@-${MV} ${OBJECTDIR}/fifo.d ${OBJECTDIR}/fifo.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/fifo.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/eeprom.p1: eeprom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eeprom.p1.d 
	@${RM} ${OBJECTDIR}/eeprom.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/eeprom.p1  eeprom.c 
	@-${MV} ${OBJECTDIR}/eeprom.d ${OBJECTDIR}/eeprom.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/eeprom.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/crc8.p1: crc8.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/crc8.p1.d 
	@${RM} ${OBJECTDIR}/crc8.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/crc8.p1  crc8.c 
	@-${MV} ${OBJECTDIR}/crc8.d ${OBJECTDIR}/crc8.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/crc8.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/hexutils.p1: hexutils.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/hexutils.p1.d 
	@${RM} ${OBJECTDIR}/hexutils.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/hexutils.p1  hexutils.c 
	@-${MV} ${OBJECTDIR}/hexutils.d ${OBJECTDIR}/hexutils.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/hexutils.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.map  -D__DEBUG=1 --debugger=realice  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     --rom=default,-7dc0-7fff --ram=default,-5f4-5ff,-f9c-f9c,-fd4-fd4,-fdb-fdf,-fe3-fe7,-feb-fef,-ffd-fff  -odist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.hex 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.map  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,-speed,+space,-debug --addrqual=ignore --mode=pro -DDEBUG -P -N255 -I"." -I"../../vscp_firmware/pic/common" -I"../../vscp_firmware/common" -I"../../vscp_software/src/vscp/common" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,+file --html --codeoffset=0x400 --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -odist/${CND_CONF}/${IMAGE_TYPE}/Frankfurt_RS-232.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/relocated
	${RM} -r dist/relocated

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
