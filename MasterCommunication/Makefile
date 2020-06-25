DependDir  = partialCompileOutput/dependLog/
ObjectDir  = partialCompileOutput/object/
SourceDir  = src/
IncludeDir = include/
BinDir     = ./
Executable = executable

CC  = gcc
CXX = g++

Includes = -Iinclude
CFLAGS   = -c -O0 -std=c98 -g -Wall $(Includes) 
CXXFLAGS = -c -O0 -std=c++17 -g -Wall $(Includes) 
LIBS     = -L. -lrt -lpthread -lboost_system
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

$(DExecutable): $(CObjects) $(CppObjects)
	$(CXX) $(LDFLAGS) $(CObjects) $(CppObjects) $(LIBS) -o $@

-include $(Depends)

$(ObjectDir)%.o: $(SourceDir)%.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

$(DependDir)%.d: $(SourceDir)%.cpp
	$(CC) -MM $(CXXFLAGS) $< > $(DependDir)$(notdir $*).d
	mv -f  $(DependDir)$(notdir $*).d  $(DependDir)$(notdir $*).d.tmp
	sed -e 's|.*:|$(ObjectDir)$(notdir $*).o $@:|' <  $(DependDir)$(notdir $*).d.tmp >  $(DependDir)$(notdir $*).d
	sed -e 's/.*://' -e 's/\\$$//' <  $(DependDir)$(notdir $*).d.tmp | fmt -1 | \
	sed -e 's/^ *//' -e 's/$$/:/' >>  $(DependDir)$(notdir $*).d
	rm -f  $(DependDir)$(notdir $*).d.tmp

$(ObjectDir)%.o: $(SourceDir)%.c
	$(CC) $(CFLAGS) $< -o $@

$(DependDir)%.d: $(SourceDir)%.c
	$(CC) -MM $(CXXFLAGS) $< > $(DependDir)$(notdir $*).d
	mv -f  $(DependDir)$(notdir $*).d  $(DependDir)$(notdir $*).d.tmp
	sed -e 's|.*:|$(ObjectDir)$(notdir $*).o $@:|' <  $(DependDir)$(notdir $*).d.tmp >  $(DependDir)$(notdir $*).d
	sed -e 's/.*://' -e 's/\\$$//' <  $(DependDir)$(notdir $*).d.tmp | fmt -1 | \
	sed -e 's/^ *//' -e 's/$$/:/' >>  $(DependDir)$(notdir $*).d
	rm -f  $(DependDir)$(notdir $*).d.tmp

.PHONY : clean
clean:
	$(RM) $(DExecutable) $(wildcard $(ObjectDir)*.o) $(wildcard $(DependDir)*.d) 


