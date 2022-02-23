DependDir  = partialCompileOutput/dependLog/
ObjectDir  = partialCompileOutput/object/
SourceDir  = src/
BinDir     = ./
Executable = executable

CC  = gcc
CXX = g++

Includes = -Iinclude -I../Library/include
CXXFLAGS = 
CFLAGS   = -c -O2 -std=c98 -g -Wall $(Includes) 
CPPFLAGS = -c -O2 -std=c++17 -g -Wall $(Includes)
LDLIBS   += -L. -lrt -lpthread -L../Library -lServoProject
LDFLAGS  = -g

######################

CSources=$(wildcard $(SourceDir)*.c)
CppSources=$(wildcard $(SourceDir)*.cpp)

CObjects   := $(patsubst $(SourceDir)%.c, $(ObjectDir)%.o, $(CSources))
CppObjects := $(patsubst $(SourceDir)%.cpp, $(ObjectDir)%.o, $(CppSources))
Depends    := $(patsubst $(ObjectDir)%.o, $(DependDir)%.d, $(CppObjects) $(CObjects))
DExecutable =$(addprefix $(BinDir),$(Executable))

.PHONY : all
all: $(DExecutable)

$(DExecutable): $(CObjects) $(CppObjects) ../Library/libServoProject.a
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(CObjects) $(CppObjects) $(LDLIBS) $(EXELINKFLAGS) -o $@

-include $(Depends)

../Library/libServoProject.a: ../Library/include/ServoProject.h ../Library/src/ServoProject.cpp
	cd ../Library && $(MAKE)

$(ObjectDir)%.o: $(SourceDir)%.cpp
	mkdir --parents $(ObjectDir)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(DependDir)%.d: $(SourceDir)%.cpp
	mkdir --parents $(DependDir)
	$(CC) -MM $(CPPFLAGS) $(CXXFLAGS) $< > $(DependDir)$(notdir $*).d
	mv -f  $(DependDir)$(notdir $*).d  $(DependDir)$(notdir $*).d.tmp
	sed -e 's|.*:|$(ObjectDir)$(notdir $*).o $@:|' <  $(DependDir)$(notdir $*).d.tmp >  $(DependDir)$(notdir $*).d
	sed -e 's/.*://' -e 's/\\$$//' <  $(DependDir)$(notdir $*).d.tmp | fmt -1 | \
	sed -e 's/^ *//' -e 's/$$/:/' >>  $(DependDir)$(notdir $*).d
	rm -f  $(DependDir)$(notdir $*).d.tmp

$(ObjectDir)%.o: $(SourceDir)%.c
	mkdir --parents $(ObjectDir)
	$(CC) $(CFLAGS) $< -o $@

$(DependDir)%.d: $(SourceDir)%.c
	mkdir --parents $(DependDir)
	$(CC) -MM $(CPPFLAGS) $(CXXFLAGS) $< > $(DependDir)$(notdir $*).d
	mv -f  $(DependDir)$(notdir $*).d  $(DependDir)$(notdir $*).d.tmp
	sed -e 's|.*:|$(ObjectDir)$(notdir $*).o $@:|' <  $(DependDir)$(notdir $*).d.tmp >  $(DependDir)$(notdir $*).d
	sed -e 's/.*://' -e 's/\\$$//' <  $(DependDir)$(notdir $*).d.tmp | fmt -1 | \
	sed -e 's/^ *//' -e 's/$$/:/' >>  $(DependDir)$(notdir $*).d
	rm -f  $(DependDir)$(notdir $*).d.tmp

.PHONY : clean
clean:
	$(RM) $(DExecutable) $(wildcard $(ObjectDir)*.o) $(wildcard $(DependDir)*.d) 
