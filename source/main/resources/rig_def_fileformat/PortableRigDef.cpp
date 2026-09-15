/* This source file is part of Rigs of Rods. */
#include "PortableRigDef.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <sstream>
namespace RoR { namespace PortableRigDef { namespace {
enum class Section { None,Globals,Nodes,Beams,Hydros,Shocks,Shocks2,Wheels,Wheels2,Engine,Brakes,Contacters,Minimass,Unknown };
struct BeamDefaults { float spring=9000000,damping=12000,deform=400000,strength=100000; };
struct BeamScale { float spring=1,damping=1,deform=1,strength=1; };
std::string Trim(const std::string&i){size_t a=0,b=i.size();while(a<b&&std::isspace((unsigned char)i[a]))++a;while(b>a&&std::isspace((unsigned char)i[b-1]))--b;return i.substr(a,b-a);}
std::string Lower(std::string s){std::transform(s.begin(),s.end(),s.begin(),[](unsigned char c){return(char)std::tolower(c);});return s;}
std::string StripComments(const std::string&i){size_t c=i.size(),s=i.find(';'),d=i.find("//");if(s!=std::string::npos)c=std::min(c,s);if(d!=std::string::npos)c=std::min(c,d);return Trim(i.substr(0,c));}
std::vector<std::string> Tokens(std::string s){for(char&c:s)if(c==','||c==':'||c=='|'||c=='\t')c=' ';std::istringstream x(s);std::vector<std::string>o;std::string t;while(x>>t)o.push_back(t);return o;}
float F(const std::string&s,float d=0){char*e=nullptr;float v=std::strtof(s.c_str(),&e);return e!=s.c_str()?v:d;} int I(const std::string&s,int d=0){char*e=nullptr;long v=std::strtol(s.c_str(),&e,10);return e!=s.c_str()?(int)v:d;}
float Resettable(const std::string&s,float def){float v=F(s,def);return v<0?def:v;}
Section S(const std::string&k){if(k=="globals")return Section::Globals;if(k=="nodes"||k=="nodes2")return Section::Nodes;if(k=="beams")return Section::Beams;if(k=="hydros")return Section::Hydros;if(k=="shocks")return Section::Shocks;if(k=="shocks2")return Section::Shocks2;if(k=="wheels")return Section::Wheels;if(k=="wheels2")return Section::Wheels2;if(k=="engine")return Section::Engine;if(k=="brakes")return Section::Brakes;if(k=="contacters")return Section::Contacters;if(k=="minimass")return Section::Minimass;return Section::Unknown;}
bool Unsupported(const std::string&k){static const char*x[]={"airbrakes","animators","axles","cameras","camerarail","cinecam","commands","commands2","exhausts","flares","flares2","flexbodies","flexbodywheels","fixes","forwardcommands","help","hooks","interaxles","lockgroups","managedmaterials","meshwheels","meshwheels2","particles","props","railgroups","ropables","ropes","rotators","rotators2","screwprops","shocks3","slidenodes","soundsources","soundsources2","submesh","ties","torquecurve","triggers","turbojets","turboprops","wings"};for(auto q:x)if(k==q)return true;return false;}
void W(Document&d,int n,const std::string&s){std::ostringstream o;o<<"line "<<n<<": "<<s;d.warnings.push_back(o.str());}
bool Has(const std::string&s,char c){return s.find(c)!=std::string::npos;}
}
Document Parse(const std::string&text){Document d;BeamDefaults bd;BeamScale bs;Section sec=Section::None;float active_minimass=50.0f;std::istringstream in(text);std::string raw;int ln=0;while(std::getline(in,raw)){++ln;auto line=StripComments(raw);if(line.empty())continue;if(d.name.empty()){d.name=line;continue;}auto t=Tokens(line);if(t.empty())continue;auto k=Lower(t[0]);if(k=="end")break;
if(k=="set_beam_defaults_scale"){if(t.size()>=5){bs.spring=F(t[1],1);bs.damping=F(t[2],1);bs.deform=F(t[3],1);bs.strength=F(t[4],1);}else W(d,ln,"set_beam_defaults_scale requires four values");continue;}
if(k=="set_beam_defaults"){if(t.size()>=5){bd.spring=Resettable(t[1],9000000)*bs.spring;bd.damping=Resettable(t[2],12000)*bs.damping;bd.deform=Resettable(t[3],400000)*bs.deform;bd.strength=Resettable(t[4],100000)*bs.strength;}else W(d,ln,"set_beam_defaults requires four values");continue;}
if(k=="set_node_defaults"){if(t.size()>=2){d.node_defaults.load_weight=Resettable(t[1],0);if(t.size()>=3)d.node_defaults.friction=Resettable(t[2],1);if(t.size()>=4)d.node_defaults.volume=Resettable(t[3],1);if(t.size()>=5)d.node_defaults.surface=Resettable(t[4],1);d.node_defaults.options=t.size()>=6?t[5]:"";}else W(d,ln,"set_node_defaults requires load weight");continue;}
if(k=="set_default_minimass"){if(t.size()>=2)active_minimass=Resettable(t[1],d.minimass);else W(d,ln,"set_default_minimass requires a value");continue;}
auto ns=S(k);if(ns!=Section::Unknown){sec=ns;continue;}if(Unsupported(k)){sec=Section::Unknown;continue;}
switch(sec){case Section::Globals:if(t.size()>=2){d.globals.present=true;d.globals.dry_mass=F(t[0]);d.globals.load_mass=F(t[1]);if(t.size()>2)d.globals.material=t[2];}else W(d,ln,"globals requires dry mass and load mass");sec=Section::None;break;
case Section::Minimass:if(!t.empty()){d.minimass=F(t[0],50);active_minimass=d.minimass;}sec=Section::None;break;
case Section::Nodes:if(t.size()>=4){Node n;n.id=t[0];n.x=F(t[1]);n.y=F(t[2]);n.z=F(t[3]);n.options=d.node_defaults.options;if(t.size()>4)n.options+=t[4];n.load_weight=d.node_defaults.load_weight;n.friction=d.node_defaults.friction;n.volume=d.node_defaults.volume;n.surface=d.node_defaults.surface;n.minimass=active_minimass;n.loaded_mass=n.load_weight>0||Has(n.options,'l')||Has(n.options,'L');n.override_mass=n.load_weight>0;if(t.size()>5&&(Has(n.options,'l')||Has(n.options,'L'))){n.load_weight=F(t[5],n.load_weight);n.override_mass=true;n.loaded_mass=true;}d.nodes.push_back(n);}else W(d,ln,"node requires id,x,y,z");break;
case Section::Beams:if(t.size()>=2){Beam b;b.node_a=t[0];b.node_b=t[1];b.spring=bd.spring;b.damping=bd.damping;b.deform=bd.deform;b.strength=bd.strength;if(t.size()>2)b.options=t[2];d.beams.push_back(b);}break;
case Section::Hydros:if(t.size()>=3){Hydro h;h.node_a=t[0];h.node_b=t[1];h.lengthening_factor=F(t[2]);if(t.size()>3)h.options=t[3];h.spring=bd.spring;h.damping=bd.damping;d.hydros.push_back(h);}break;
case Section::Shocks:case Section::Shocks2:if(t.size()>=7){Shock s;s.node_a=t[0];s.node_b=t[1];s.spring=F(t[2]);s.damping=F(t[3]);if(sec==Section::Shocks){s.short_bound=F(t[4]);s.long_bound=F(t[5]);s.precompression=F(t[6],1);}else if(t.size()>=13){s.short_bound=F(t[10]);s.long_bound=F(t[11]);s.precompression=F(t[12],1);}d.shocks.push_back(s);}break;
case Section::Wheels:if(t.size()>=12){Wheel w;w.tire_radius=F(t[0]);w.width=F(t[1]);w.num_rays=I(t[2]);w.axis_node_0=t[3];w.axis_node_1=t[4];w.rigidity_node=t[5];w.braking=I(t[6]);w.propulsion=I(t[7]);w.reference_arm_node=t[8];w.mass=F(t[9]);w.tire_spring=F(t[10]);w.tire_damping=F(t[11]);d.wheels.push_back(w);}break;
case Section::Wheels2:if(t.size()>=15){Wheel w;w.wheels2=true;w.rim_radius=F(t[0]);w.tire_radius=F(t[1]);w.width=F(t[2]);w.num_rays=I(t[3]);w.axis_node_0=t[4];w.axis_node_1=t[5];w.rigidity_node=t[6];w.braking=I(t[7]);w.propulsion=I(t[8]);w.reference_arm_node=t[9];w.mass=F(t[10]);w.rim_spring=F(t[11]);w.rim_damping=F(t[12]);w.tire_spring=F(t[13]);w.tire_damping=F(t[14]);d.wheels.push_back(w);}break;
case Section::Engine:if(t.size()>=4){d.engine.present=true;d.engine.shift_down_rpm=F(t[0]);d.engine.shift_up_rpm=F(t[1]);d.engine.torque=F(t[2]);d.engine.differential_ratio=F(t[3],1);d.engine.gear_ratios.clear();for(size_t i=4;i<t.size();++i)d.engine.gear_ratios.push_back(F(t[i]));}break;
case Section::Brakes:if(!t.empty()){d.brakes.present=true;d.brakes.service_force=F(t[0],30000);if(t.size()>1)d.brakes.parking_force=F(t[1],-1);}break;case Section::Contacters:d.contacters.push_back(t[0]);break;default:break;}}
return d;}
} }
