############################################################################
# boards/Board.mk
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# Files laid over the board's ROMFS at CONFIG_ETC_ROMFSMOUNTPT by whoever
# builds NuttX: ETC_OVERLAY, given on make's command line (it reaches the
# board's build through MAKEFLAGS), names a directory, or a symbolic link
# to one, by an absolute path, whose tree is the ROMFS's (init.d/x.rc for
# /etc/init.d/x.rc).  Its *.rc files are preprocessed as the board's are,
# the others copied as they are; dotfiles and dot directories are left
# out.  A file the board has too is the overlay's, except the passwd file
# the board may generate.  It needs CONFIG_ETC_ROMFS.  A file removed from
# the overlay stays in the ROMFS until "make clean".  Make only: the CMake
# build has hooks of its own (cmake/nuttx_add_romfs.cmake).

ifneq ($(ETC_OVERLAY),)
ifeq ($(filter /%,$(ETC_OVERLAY)),)
$(error ETC_OVERLAY: $(ETC_OVERLAY) is not an absolute path)
endif
override ETC_OVERLAY := $(abspath $(ETC_OVERLAY))
ifeq ($(wildcard $(ETC_OVERLAY)/.),)
$(error ETC_OVERLAY: $(ETC_OVERLAY) is not a directory)
endif
ifeq ($(CONFIG_ETC_ROMFS),y)
ETCMOUNT     := $(patsubst "%",%,$(CONFIG_ETC_ROMFSMOUNTPT))
OVERLAYFILES := $(patsubst $(ETC_OVERLAY)/%,%,$(shell find -H $(ETC_OVERLAY) -mindepth 1 -name '.*' -prune -o -type f -print 2> /dev/null))
OVERLAYRCS   := $(filter %.rc,$(OVERLAYFILES))
OVERLAYRAWS  := $(filter-out %.rc,$(OVERLAYFILES))
endif
endif

ifneq ($(RCSRCS)$(RCRAWS)$(OVERLAYFILES),)
ETCDIR := etctmp
ETCSRC := $(ETCDIR:%=%.c)

CSRCS += $(ETCSRC)

OVERLAYOBJS = $(OVERLAYRCS:%=$(ETCDIR)$(ETCMOUNT)$(DELIM)%)
RCOBJS = $(filter-out $(OVERLAYOBJS),$(RCSRCS:%=$(ETCDIR)$(DELIM)%))

$(RCOBJS): $(ETCDIR)$(DELIM)%: %
	$(Q) mkdir -p $(dir $@)
	$(call PREPROCESS, $<, $@)

# Preprocessed again when the configuration changes, as they test it

ifneq ($(OVERLAYOBJS),)
$(OVERLAYOBJS): $(ETCDIR)$(ETCMOUNT)$(DELIM)%: $(ETC_OVERLAY)$(DELIM)% $(TOPDIR)$(DELIM).config
	$(Q) mkdir -p $(dir $@)
	$(call PREPROCESS, $<, $@)
endif

$(ETCSRC): $(foreach raw,$(RCRAWS), $(if $(wildcard $(BOARD_DIR)$(DELIM)src$(DELIM)$(raw)), $(BOARD_DIR)$(DELIM)src$(DELIM)$(raw), $(if $(wildcard $(BOARD_COMMON_DIR)$(DELIM)$(raw)), $(BOARD_COMMON_DIR)$(DELIM)$(raw), $(BOARD_DIR)$(DELIM)src$(DELIM)$(raw)))) $(RCOBJS) $(OVERLAYOBJS) $(OVERLAYRAWS:%=$(ETC_OVERLAY)$(DELIM)%) $(TOPDIR)$(DELIM).config
	$(foreach raw, $(RCRAWS), \
	  $(shell rm -rf $(ETCDIR)$(DELIM)$(raw)) \
	  $(shell mkdir -p $(dir $(ETCDIR)$(DELIM)$(raw))) \
	  $(shell cp -rfp $(if $(wildcard $(BOARD_DIR)$(DELIM)src$(DELIM)$(raw)), $(BOARD_DIR)$(DELIM)src$(DELIM)$(raw), $(if $(wildcard $(BOARD_COMMON_DIR)$(DELIM)$(raw)), $(BOARD_COMMON_DIR)$(DELIM)$(raw), $(BOARD_DIR)$(DELIM)src$(DELIM)$(raw))) $(ETCDIR)$(DELIM)$(raw)))
	$(Q) $(foreach raw,$(OVERLAYRAWS),mkdir -p $(dir $(ETCDIR)$(ETCMOUNT)$(DELIM)$(raw)) && \
	  cp -fp $(ETC_OVERLAY)$(DELIM)$(raw) $(ETCDIR)$(ETCMOUNT)$(DELIM)$(raw) &&) true
ifeq ($(CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE),y)
	$(Q) set -e; \
	mkdir -p $(ETCDIR)$(DELIM)$(CONFIG_ETC_ROMFSMOUNTPT); \
	$(TOPDIR)$(DELIM)tools$(DELIM)board_romfs_mkpasswd.sh \
		$(TOPDIR) $(ETCDIR)$(DELIM).romfs_passwd.txt \
		$(TOPDIR)$(DELIM)tools$(DELIM)mkpasswd$(HOSTEXEEXT) \
		$(ETCDIR)$(DELIM)$(CONFIG_ETC_ROMFSMOUNTPT)$(DELIM)passwd \
		--user $(CONFIG_BOARD_ETC_ROMFS_PASSWD_USER) \
		--uid $(CONFIG_BOARD_ETC_ROMFS_PASSWD_UID) \
		--gid $(CONFIG_BOARD_ETC_ROMFS_PASSWD_GID) \
		--home $(CONFIG_BOARD_ETC_ROMFS_PASSWD_HOME)
endif
	$(Q) genromfs -f romfs.img -d $(ETCDIR)$(DELIM)$(CONFIG_ETC_ROMFSMOUNTPT) -V "NSHInitVol"
	$(Q) echo "#include <nuttx/compiler.h>" > $@
	$(Q) xxd -i romfs.img | sed -e "s/^unsigned char/const unsigned char aligned_data(4)/g" >> $@
	$(Q) rm romfs.img
endif

ifneq ($(ZDSVERSION),)
AOBJS = $(ASRCS:.S=$(OBJEXT))
else
AOBJS = $(ASRCS:$(ASMEXT)=$(OBJEXT))
endif
COBJS = $(CSRCS:.c=$(OBJEXT))
CXXOBJS = $(CXXSRCS:.cxx=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

SCHEDSRCDIR = $(TOPDIR)$(DELIM)sched
ARCHSRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src
ifneq ($(CONFIG_ARCH_FAMILY),)
  ARCH_FAMILY = $(patsubst "%",%,$(CONFIG_ARCH_FAMILY))
endif

CFLAGS += ${INCDIR_PREFIX}"$(SCHEDSRCDIR)"
CFLAGS += ${INCDIR_PREFIX}"$(ARCHSRCDIR)$(DELIM)chip"
ifneq ($(CONFIG_ARCH_SIM),y)
  CFLAGS += ${INCDIR_PREFIX}"$(ARCHSRCDIR)$(DELIM)common"
endif
ifneq ($(ARCH_FAMILY),)
  CFLAGS += ${INCDIR_PREFIX}"$(ARCHSRCDIR)$(DELIM)$(ARCH_FAMILY)"
endif

all: libboard$(LIBEXT)

ifneq ($(ZDSVERSION),)
$(ASRCS) $(HEAD_ASRC): %$(ASMEXT): %.S
	$(Q) $(CPP) $(CPPFLAGS) $(call CONVERT_PATH,$<) -o $@.tmp
	$(Q) cat $@.tmp | sed -e "s/^#/;/g" > $@
	$(Q) rm $@.tmp
endif

$(AOBJS): %$(OBJEXT): %$(ASMEXT)
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS) $(LINKOBJS): %$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

libboard$(LIBEXT): $(OBJS) $(CXXOBJS)
	$(call ARCHIVE, $@, $(OBJS) $(CXXOBJS))

.depend: Makefile $(SRCS) $(CXXSRCS) $(RCSRCS) $(TOPDIR)$(DELIM).config
ifneq ($(ZDSVERSION),)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
else
	$(Q) $(MKDEP) $(DEPPATH) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
ifneq ($(CXXSRCS),)
	$(Q) $(MKDEP) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(CXXSRCS) >>Make.dep
endif
ifneq ($(RCSRCS),)
	$(Q) $(MKDEP) $(DEPPATH) "$(CPP)" --obj-path . -- $(CPPFLAGS) -- $(RCSRCS) >>Make.dep
endif
	$(Q) touch $@

depend: .depend

context::

clean::
	$(call DELFILE, libboard$(LIBEXT))
	$(call DELFILE, $(ETCSRC))
	$(call DELDIR, $(ETCDIR))
	$(call CLEAN)

distclean:: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
