#include "generate.hpp"
#include "bsp.hpp"
#include "gma.hpp"
#include <cfenv>
#include <clocale>
#include <string_view>

namespace {
using namespace navgen;
void usage(std::ostream& out) {
    out<<"usage: navgen <map.bsp> [--game DIR] [--out FILE] [--seeds FILE]\n"
         "              [--slope-limit F] [--threads N] [--profile sdk|merged]\n"
         "              [--seed-discovery none|entities] [--ladders] [--missing-phy skip|bbox]\n";
}
bool digit(char c) { return c>='0'&&c<='9'; }
[[noreturn]] void invalid_number(const std::string& name,const std::string& value) {
    throw Error("invalid value for "+name+": "+value,"UsageError");
}
int thread_number(const std::string& value) {
    try {
        const auto parsed=python_integer(value).clamped();
        // Python accepts integers larger than C++ int. Preserve their later
        // thread-range failure, rather than reporting a lexical usage error.
        return static_cast<int>(std::clamp<std::int64_t>(parsed,std::numeric_limits<int>::min(),std::numeric_limits<int>::max()));
    } catch(const Error& error) {
        if(error.kind!="ValueError")throw;
        invalid_number("--threads",value);
    }
}
double slope_number(const std::string& value) {
    try {return python_float(value);}
    catch(const Error& error) {
        if(error.kind!="ValueError")throw;
        invalid_number("--slope-limit",value);
    }
}
bool negative_number(const std::string& value) {
    if(value.size()<2||value[0]!='-')return false;
    auto normalised=python_number_text(value);
    if(!normalised)return false;
    auto& text=*normalised;
    // argparse's regular-expression end anchor accepts a final newline.
    if(text.ends_with('\n'))text.pop_back();
    std::size_t at=1;while(at<text.size()&&digit(text[at]))++at;
    if(at==text.size())return at>1;
    if(text[at++]!='.')return false;
    const auto begin=at;while(at<text.size()&&digit(text[at]))++at;
    return at>begin&&at==text.size();
}
struct Argument {
    enum class Kind { positional,option,unknown,separator } kind;
    std::string raw,name;
    std::optional<std::string> value;
};
std::vector<Argument> arguments(int argc,char** argv) {
    const std::vector<std::string> names{"--help","--game","--out","--seeds","--slope-limit","--threads","--profile","--seed-discovery","--ladders","--missing-phy","--dump-stages"};
    std::vector<Argument> result;bool positional=false;
    for(int i=1;i<argc;++i) {
        const std::string raw=argv[i];
        if(positional) {result.push_back({Argument::Kind::positional,raw,{},{}});continue;}
        if(raw=="--") {result.push_back({Argument::Kind::separator,raw,{},{}});positional=true;continue;}
        if(raw=="-h") {result.push_back({Argument::Kind::option,raw,"--help",{}});continue;}
        if(raw.starts_with("--")) {
            const auto equal=raw.find('=');auto name=raw.substr(0,equal);
            if(std::find(names.begin(),names.end(),name)==names.end()) {
                std::vector<std::string> matches;
                for(const auto& option:names)if(option.starts_with(name))matches.push_back(option);
                if(matches.size()>1)throw Error("ambiguous option: "+name,"UsageError");
                if(matches.size()==1)name=matches.front();
                else {result.push_back({raw.find(' ')==std::string::npos?Argument::Kind::unknown:Argument::Kind::positional,raw,{},{}});continue;}
            }
            std::optional<std::string> value;
            if(equal!=std::string::npos)value=raw.substr(equal+1);
            result.push_back({Argument::Kind::option,raw,std::move(name),std::move(value)});
        } else if(raw.starts_with("-h")) {
            if(std::all_of(raw.begin()+1,raw.end(),[](char c){return c=='h';}))result.push_back({Argument::Kind::option,raw,"--help",{}});
            else result.push_back({Argument::Kind::option,raw,"--help",raw.substr(2)});
        } else {
            const bool option=raw.size()>1&&raw[0]=='-'&&!negative_number(raw)&&raw.find(' ')==std::string::npos;
            result.push_back({option?Argument::Kind::unknown:Argument::Kind::positional,raw,{},{}});
        }
    }
    return result;
}
fs::path path_argument(const std::string& value) {return path_from_utf8(value.empty()?".":value);}
}
int run_main(int argc,char** argv) {
    using namespace navgen;std::setlocale(LC_ALL,"C");std::fesetround(FE_TONEAREST);
    Options options;options.game=find_game();options.threads=logical_cores();
    try {
        const auto parsed=arguments(argc,argv);bool has_map=false;std::vector<std::string> extra;
        for(std::size_t i=0;i<parsed.size();++i) {
            const auto& argument=parsed[i];
            if(argument.kind==Argument::Kind::separator)continue;
            if(argument.kind==Argument::Kind::unknown) {extra.push_back(argument.raw);continue;}
            if(argument.kind==Argument::Kind::positional) {
                if(has_map)extra.push_back(argument.raw);
                else {options.map=path_argument(argument.raw);has_map=true;}
                continue;
            }
            const auto& name=argument.name;
            if(name=="--help"||name=="--ladders") {
                if(argument.value)throw Error(name+" does not take a value","UsageError");
                if(name=="--help") {usage(std::cout);std::cout<<"Generate a Source v16/0 NAV offline with hiding analysis and optional conservative merging.\n";return 0;}
                options.ladders=true;continue;
            }
            auto value=argument.value;
            if(!value) {
                if(i+1==parsed.size()||parsed[i+1].kind!=Argument::Kind::positional)throw Error("expected a value after "+name,"UsageError");
                value=parsed[++i].raw;
            }
            if(name=="--game")options.game=path_argument(*value);
            else if(name=="--out")options.out=path_argument(*value);
            else if(name=="--seeds")options.seeds=path_argument(*value);
            else if(name=="--dump-stages")options.dump_stages=path_argument(*value);
            else if(name=="--profile") {if(*value!="sdk"&&*value!="merged")throw Error("invalid --profile choice: "+*value,"UsageError");options.profile=*value;}
            else if(name=="--seed-discovery") {if(*value!="none"&&*value!="entities")throw Error("invalid --seed-discovery choice: "+*value,"UsageError");options.seed_discovery=*value;}
            else if(name=="--missing-phy") {if(*value!="skip"&&*value!="bbox")throw Error("invalid --missing-phy choice: "+*value,"UsageError");options.missing_phy=*value;}
            else if(name=="--threads")options.threads=thread_number(*value);
            else if(name=="--slope-limit")options.slope=slope_number(*value);
        }
        if(!has_map)throw Error("the map argument is required","UsageError");
        if(!extra.empty())throw Error("unrecognised argument: "+extra.front(),"UsageError");
    } catch(const Error& error) {
        usage(std::cerr);std::cerr<<"navgen: error: "<<error.what()<<'\n';return 2;
    }
    try {configure_threads(options.threads);std::cout<<generate(options).dump()<<std::endl;return 0;}
    catch(const Error& error) {std::cerr<<"navgen failed: "<<error.kind<<": "<<error.what()<<std::endl;}
    catch(const fs::filesystem_error& error) {std::cerr<<"navgen failed: OSError: "<<error.what()<<std::endl;}
    catch(const std::bad_alloc& error) {std::cerr<<"navgen failed: MemoryError: "<<error.what()<<std::endl;}
    catch(const std::exception& error) {std::cerr<<"navgen failed: ValueError: "<<error.what()<<std::endl;}
    return 1;
}

#ifdef _WIN32
int wmain(int argc,wchar_t** argv) {
    using namespace navgen;
    try {
        std::vector<std::string> text;text.reserve(static_cast<std::size_t>(argc));
        for(int i=0;i<argc;++i) {
            std::vector<std::uint32_t> points;
            const std::wstring_view wide=argv[i];
            for(std::size_t at=0;at<wide.size();++at) {
                auto point=static_cast<std::uint32_t>(wide[at]);
                if(point>=0xd800&&point<=0xdbff) {
                    if(at+1==wide.size()||wide[at+1]<0xdc00||wide[at+1]>0xdfff)throw Error("Invalid UTF-16 command line","UnicodeEncodeError");
                    point=0x10000+((point-0xd800)<<10)+(static_cast<std::uint32_t>(wide[++at])-0xdc00);
                } else if(point>=0xdc00&&point<=0xdfff)throw Error("Invalid UTF-16 command line","UnicodeEncodeError");
                points.push_back(point);
            }
            text.push_back(unicode_utf8(points));
        }
        std::vector<char*> pointers;pointers.reserve(text.size());
        for(auto& value:text)pointers.push_back(value.data());
        return run_main(argc,pointers.data());
    } catch(const Error& error) {
        std::cerr<<"navgen failed: "<<error.kind<<": "<<error.what()<<std::endl;return 1;
    } catch(const std::exception& error) {
        std::cerr<<"navgen failed: ValueError: "<<error.what()<<std::endl;return 1;
    }
}
#else
int main(int argc,char** argv) {return run_main(argc,argv);}
#endif
