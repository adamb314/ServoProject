DependDir  = partialCompileOutput/dependLog/
ObjectDir  = partialCompileOutput/object/
SourceDir  = src/
IncludeDir = include/
BinDir     = ./
Library = libServoProject.a

CC  = gcc
CXX = g++

Includes = -Iinclude
CFLAGS   = -c -O2 -std=c98 -g -Wall $(Includes) 
CXXFLAGS = -c -O2 -std=c++17 -g -Wall $(Includes)
LIBS     = 
LDFLAGS  = -g

######################

CSources=$(wildcard $(SourceDir)*.c)
CppSources=$(wildcard $(SourceDir)*.cpp)

CObjects   := $(patsubst $(SourceDir)%.c, $(ObjectDir)%.o, $(CSources))
CppObjects := $(patsubst $(SourceDir)%.cpp, $(ObjectDir)%.o, $(CppSources))
Depends    := $(patsubst $(ObjectDir)%.o, $(DependDir)%.d, $(CppObjects) $(CObjects))
DLibrary =$(addprefix $(BinDir),$(Library))

.PHONY : all
all: $(DLibrary)

libServoProject.a: $(CObjects) $(CppObjects)
	ar rcs $@ $(CObjects) $(CppObjects) 

-include $(Depends)

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
	$(RM) $(DLibrary) $(ObjectDir)* $(DependDir)*
