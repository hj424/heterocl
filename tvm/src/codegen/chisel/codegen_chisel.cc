/*!
 *  Copyright (c) 2017 by Contributors
 * \file codegen_chisel.cc
 */

#include <tvm/build_module.h>
#include <tvm/ir_pass.h>
#include <vector>
#include <string>
#include <regex>
#include "./codegen_chisel.h"
#include "../build_common.h"

namespace TVM {
namespace codegen {

using namespace ir;

void CodeGenCHISEL::Init(bool output_ssa) {
  print_ssa_form_ = output_ssa;
  LOG(INFO) << "checkpoint init";
}

void CodeGenCHISEL::InitFuncState(LoweredFunc f) {
  LOG(INFO) << "checkpoint InitFuncState";
  alloc_storage_scope_.clear();
  handle_data_type_.clear();
  CodeGenSourceBase::ClearFuncState();
}
void CodeGenCHISEL::AddFunction(LoweredFunc f,
        str2tupleMap<std::string, Type> map_arg_type) {
  LOG(INFO) << "checkpoint AddFunction";
  // clear previous generated state.
  this->InitFuncState(f);
  // skip the first underscore, so SSA variable starts from _1
  GetUniqueName("_");
  // add to alloc buffer type.
  for (const auto & kv : f->handle_data_type) {
    RegisterHandleType(kv.first.get(), kv.second.type());
  }
  // print out package and improted libraries 
  this->stream << "// Chisel xcel: " << f->name <<"\n";
  this->stream << "package " << f->name << "\n\n";
  // store function scope
  int circuit_scope = this->BeginScope();
  int circuit_scope_r = this->BeginScope_body();
  LOG(INFO) << "check function scope";
  this->stream << "// import chisel libraries\n";
  this->stream << "import chisel3._\n";
  this->stream << "import chisel3.util._\n\n";
  int module_scope = this->BeginScope();
  int module_scope_r = this->BeginScope_body();
  //input varaibles
  for (size_t i = 0; i < f->args.size(); ++i) {
    Var v = f->args[i];
    std::string vid = AllocVarID(v.get());
    is_input[v.get()] = true;
    LOG(INFO) << vid;
  }
  // checkpoint: print out current stream and body IR
  LOG(INFO) << stream.str();
  LOG(INFO) << f->body;

  // traverse the function body to figure out the direction of ports
  // update unordered_map is_input
  this->PrintStmt(f->body);

  // print out the class header
  stream << "// Main function body \n";
  stream << "class " << f->name << " extends Module { \n";
  this -> PrintIndent();
  stream << "val io = IO (new Bundle { \n";

  // specify && print out input/output ports
  for (size_t i = 0; i < f->args.size(); ++i) {
    Var v = f->args[i];
    std::string vid = GetVarID(v.get());
    std::ostringstream arg_name;
    // detect input/output ports
    if (map_arg_type.find(vid) == map_arg_type.end()) {
      LOG(WARNING) << vid << " type not found\n";
      arg_name << ' ' << vid << " : ";
      PrintType(v.type(), arg_name);
    }
    else {
      auto arg = map_arg_type[vid];
      if (is_input[v.get()] == true) {
        arg_name << "        val " << std::get<0>(arg) << " = Flipped(Decoupled(";
      }
      else {
        arg_name << "        val " << std::get<0>(arg) << " = Decoupled(";
      }
      LOG(INFO) <<std::get<0>(arg) << ";"<< std::get<1>(arg) << "========";
      PrintType(std::get<1>(arg), arg_name);
    }
    LOG(INFO) << "PRINTING input output ports";
    // print out input/output ports
    stream << arg_name.str();
    if (is_input[v.get()] == true) {
      stream << "))\n";
    }
    else {
      stream << ")\n";
    }
    LOG(INFO) << stream.str();
  }
  stream << "    }\n\n";
  
  // print out local vars
  std::ostringstream stream_local_vars;
  this -> PrintIndent_stream(stream_local_vars);
  stream_local_vars << "// local variables\n";
  this -> PrintIndent_stream(stream_local_vars);
  stream_local_vars << "val state = Reg(UInt(8.W))\n";
  this -> PrintIndent_stream(stream_local_vars);
  stream_local_vars << "val cnt = Reg(UInt(8.W))\n"; // update later: should be hinted from the IR
  // traverse the is_input map to find the # of output ports
  for (size_t i = 0; i < f->args.size(); ++i) {
    Var v = f->args[i];
    std::string vid = GetVarID(v.get());
    auto arg = map_arg_type[vid];
    if (is_input[v.get()] == false) {
      this -> PrintIndent_stream(stream_local_vars);
      stream_local_vars << "val " << std::get<0>(arg) << " = Reg(";
      PrintType(std::get<1>(arg), stream_local_vars);
      stream_local_vars << "))\n";
    }
  }
  stream_local_vars << "\n";
  stream << stream_local_vars.str();
  
  // print out fsm
  std::ostringstream stream_fsm;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "// FSM\n";
  this -> PrintIndent_stream(stream_fsm);
  // idle
  stream_fsm << "when (state === 0.U) {\n";
  indent_current += 2;
  for ( size_t i = 0; i < f->args.size(); ++i ){
    Var v = f->args[i];
    std::string vid = GetVarID(v.get());
    auto arg = map_arg_type[vid];
    this -> Print_init_valrdy(stream_fsm, std::get<0>(arg), is_input[v.get()]);
  }
  this -> PrintIndent_stream(stream_fsm);
 // create when statement with multiple valrdy signals
  bool later_iter = false;
  stream_fsm << "when (";
  for ( size_t i = 0; i < f->args.size(); ++i ){
    Var v = f->args[i];
    std::string vid = GetVarID(v.get());
    auto arg = map_arg_type[vid];
    if (is_input[v.get()]) {
      if (later_iter) {
        stream_fsm << " || ";
      }
      later_iter = true;
      stream_fsm << std::get<0>(arg) << ".valid";
    }
  }
  stream_fsm << ") {\n";
  indent_current += 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "state := 1.U\n";
  indent_current -= 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "}\n";
  // reset
  indent_current -= 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "}.elsewhen (state === 1.U) {\n";
  indent_current += 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "state := 2.U\n";
  this -> PrintIndent_stream(stream_fsm); // update later: detect init values in IR level
  stream_fsm << "b := io.a.bits\n";       // update later: hard coded now
  indent_current -= 2;
  this -> PrintIndent_stream(stream_fsm);
  // compute
  stream_fsm << "}.otherwise{\n";
  indent_current += 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "when (cnt === 5.U) {\n";  // update later: should be info from IR
  indent_current += 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "io.";
  for ( size_t i = 0; i < f->args.size(); ++i ){
    Var v = f->args[i];
    std::string vid = GetVarID(v.get());
    auto arg = map_arg_type[vid];
    if (is_input[v.get()] == false) {
      stream_fsm << std::get<0>(arg) << ".valid := true.B\n";
    }
  }
  indent_current -= 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "}.otherwise {\n";
  indent_current += 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "io.";
  for ( size_t i = 0; i < f->args.size(); ++i ){
    Var v = f->args[i];
    std::string vid = GetVarID(v.get());
    auto arg = map_arg_type[vid];
    if (is_input[v.get()] == false) {
      stream_fsm << std::get<0>(arg) << ".valid := false.B\n";
    }
  }
  indent_current -= 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "}\n";
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "when (";
  later_iter = false;
  for ( size_t i = 0; i < f->args.size(); ++i ){
    Var v = f->args[i];
    std::string vid = GetVarID(v.get());
    auto arg = map_arg_type[vid];
    if (is_input[v.get()] == false) {
      if (later_iter) {
        stream_fsm << " && ";
      }
      later_iter = true;
      stream_fsm << std::get<0>(arg) << ".ready";
    }
  }
  stream_fsm << ") {\n";
  indent_current += 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "state := 0.U\n";
  indent_current -= 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "}\n";
  
  // enclose "}"
  indent_current -= 2;
  this -> PrintIndent_stream(stream_fsm);
  stream_fsm << "}\n";
  
  stream << stream_fsm.str(); 
  
  // print function body
  stream << stream_body.str();
  stream << "}\n";
  
  //deal with the final connections of registers
  this->EndScope(module_scope);
  this->EndScope_body(module_scope_r);
  this->EndScope(circuit_scope);
  this->EndScope_body(circuit_scope_r);
  //might have multiple modules
}

std::string CodeGenCHISEL::Finish() {
  LOG(INFO) << "checkpoint finish";
  return decl_stream.str() + stream.str();
}
/*
void CodeGenCHISEL::PrintSSAAssign(
    const std::string& target, const std::string& src, Type t) {
  LOG(INFO) << "checkpoint PrintSSAAssign";
  PrintType(t, stream);
  stream << ' ' << target << " = ";
  if (src.length() > 3 &&
      src[0] == '(' && src[src.length() - 1] == ')') {
    stream << src.substr(1, src.length() - 2);
  } else {
    stream << src;
  }
  stream << ";\n";
}*/

// print out the valrdy signal for init state
void CodeGenCHISEL::Print_init_valrdy(std::ostream& os, std::string var, bool is_input) {
  LOG(INFO) << "Print_init_valrdy";
  this -> PrintIndent_stream(os);
  if (is_input) {
    os << "io." << var << ".ready := true.B\n";
  }
  else {
    os << "io." << var << ".valid := false.B\n";
  }
}

// helper function: print indent
void CodeGenCHISEL::PrintIndent_stream(std::ostream& os) {
  LOG(INFO) << "PrintIndent_stream";
  for (int i = 0; i < indent_current; ++i) {
    os << ' ';
  }
}


void CodeGenCHISEL::PrintIndent_body() {
  LOG(INFO) << "PrintIndent_body";
  for (int i = 0; i < indent_body; ++i) {
    this->stream_body << ' ';
  }
}

int CodeGenCHISEL::BeginScope_body() {
  LOG(INFO) << "checkpoint BeginScope_body";
  int sid = static_cast<int>(scope_mark_body.size());
  scope_mark_body.push_back(true);
  indent_body += 2;
  return sid;
}

void CodeGenCHISEL::EndScope_body(int scope_id) {
  LOG(INFO) << "checkpoint EndScope_body";
  scope_mark_body[scope_id] = false;
  indent_body -= 2;
}

void CodeGenCHISEL::PrintExpr(const Expr& n, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint PrintExpr";
  LOG(INFO) << stream_body.str();
  if (print_ssa_form_) {
    std::ostringstream temp;
    VisitExpr(n, temp);
    os << SSAGetID(temp.str(), n.type());
  } else {
    VisitExpr(n, os);
  }
}

void CodeGenCHISEL::PrintType(Type t, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint PrintType";
  CHECK_EQ(t.lanes(), 1)
      << "do not yet support vector types";
  if (t.is_handle()) {
    os << "void*"; return;
  }
  if (t.is_float()) {
    if (t.bits() == 32) {
      os << "float"; return;
    }
    if (t.bits() == 64) {
      os << "double"; return;
    }
  } else if (t.is_uint()) {
    switch (t.bits()) {
      case 8: case 16: case 32: case 64: {
        os << "UInt(" << t.bits() << ".W)"; return;
      }
      case 1: os << "SInt"; return;
    }
  } else if (t.is_int()) {
    switch (t.bits()) {
      case 8: case 16: case 32: case 64: {
        os << "SInt(" << t.bits() << ".W)";  return;
      }
    }
  }
  LOG(FATAL) << "Cannot convert type " << t << " to C type";
}


inline void PrintConst(const UIntImm* op, std::ostream& os, CodeGenCHISEL* p) { // NOLINT(*)
  LOG(INFO) << "checkpoint PrintConst UINT";
  std::ostringstream temp;
  temp << "UInt(" << op->value << ")";
  p->MarkConst(temp.str());
  os << temp.str();
}

inline void PrintConst(const IntImm* op, std::ostream& os, CodeGenCHISEL* p) { // NOLINT(*)
  LOG(INFO) << "checkpoint PrintConst INT";
  std::ostringstream temp;
  temp << "SInt(" << op->value << ")";
  p->MarkConst(temp.str());
  os << temp.str();
}


bool CodeGenCHISEL::HandleTypeMatch(const Variable* buf_var, Type t) const {
  LOG(INFO) << "checkpoint HandleTypeMatch";
  auto it = handle_data_type_.find(buf_var);
  if (it == handle_data_type_.end()) return false;
  return it->second == t;
}

void CodeGenCHISEL::RegisterHandleType(const Variable* buf_var, Type t) {
  LOG(INFO) << "checkpoint RegisterHandleType";
  auto it = handle_data_type_.find(buf_var);
  if (it == handle_data_type_.end()) {
    handle_data_type_[buf_var] = t;
  } else {
    CHECK(it->second == t)
        << "conflicting buf var type";
  }
}


void CodeGenCHISEL::VisitExpr_(const UIntImm *op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_ UInt";
  PrintConst(op, os, this);
}

void CodeGenCHISEL::VisitExpr_(const IntImm *op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_Int";
  PrintConst(op, os, this);
}


template<typename T>
inline void PrintBinaryExpr(const T* op,
                            const char *opstr,
                            std::ostream& os,  // NOLINT(*)
                            CodeGenCHISEL* p) {
  LOG(INFO) << "checkpoint PrintBinaryExpr";
  if (op->type.lanes() == 1) {// what is this
      os << opstr << '(';
      p->PrintExpr(op->a, os);
      os << ", ";
      p->PrintExpr(op->b, os);
      os << ')';
  } else {
    LOG(FATAL) << "Vector not supported";
  }
}

// Print a reference expression to a buffer.
std::string CodeGenCHISEL::GetWire(
    Type t, const Variable* v, std::ostream& os) {
  LOG(INFO) << "checkpoint GetWire";
  std::ostringstream wire;
  std::string vid = GetVarID(v);
  auto wire_reg_auto = vid_wire_reg.find(v);
  if (wire_reg_auto == vid_wire_reg.end()){
    wire << vid;
    wire << "_0";
    vid_wire_reg[v] = "0";
    return wire.str();
  }
  auto reg_init = wire_f_list.find(v);
  std::string wire_reg = wire_reg_auto->second;
  if ( in_for == true && reg_init == wire_f_list.end() ){//first store in For. there was a store before For wire_reg.compare("_r") != 0){
    // !!!here
    this -> PrintIndent_body();
    os << "reg " << vid << "_r : ";
    this -> PrintType(t,os);
    os << ", clk\n";
    //this -> PrintIndent_body();
    //os << "wire " << vid << "_f : ";
    //this -> PrintType(t,os);
    os << "\n";
    this -> PrintIndent_body();
    os << "when n1 : \n";
    this -> PrintIndent_body();
    this -> PrintIndent_body();
    os << vid << "_r <=  add(";
    os << vid << "_" << wire_reg <<", ";
    os << vid << "_f)\n";
    this -> PrintIndent_body();
    
    //, initialize reg with reset and connect to rightside
    // reg out_r: UInt<32>, clk ;;initialize reg for every variable and input/output port in its FIRST Store. -> Hashmap reg_set[out_r] = 1
    //
    
    wire << vid << "_r";
    wire_f_list[v] = wire_reg;
    vid_wire_reg[v] = "r";
    
  }
  else{
    
    int new_wire_num;
    if ( wire_reg.compare("r") == 0 ){ //the last wire was connected to the register
      new_wire_num = std::stoi(wire_f_list[v]) + 1;
    }
    else new_wire_num = std::stoi(wire_reg) + 1;
    wire << vid;
    wire << "_" << std::to_string(new_wire_num);
    vid_wire_reg[v] = std::to_string(new_wire_num);
    if(reg_init != wire_f_list.end()) wire_f_list[v] = std::to_string(new_wire_num);
    
  }
  return wire.str();
}


//implement Visit Expr
void CodeGenCHISEL::VisitExpr_(const Variable *op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_ Var"; 
  auto wire_reg_auto = vid_wire_reg.find(op);
  if (wire_reg_auto != vid_wire_reg.end()) os << GetWire(op->type,op,os);
  else {
    os << GetVarID(op);
    LOG(INFO) << GetVarID(op);
  }
  LOG(INFO) << "vid_wire_reg: size" << vid_wire_reg.size() << "++++++++++!!!!!!!!!"; 
  if (in_let) var_to_arg[VARkey_to_arg] = op;
}

void CodeGenCHISEL::VisitExpr_(const Add *op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_ Add";
  PrintBinaryExpr(op, "add", os, this);
/*  this->PrintIndent_body();
  const Variable *a_id = (op->a).get();
  auto op_a_find = wire_f_list.find(a_id);
  if (op_a_find != wire_f_list.end()){
    std::string op_a = PrintExpr(op->a) + "_r";
  }
  else{
    std::string op_a = PrintExpr(op->a);
  }
  os <<"add(" << op_a << "," << PrintExpr(op->b) <<")\n";*/
}


void CodeGenCHISEL::VisitStmt_(const LetStmt* op) {
  LOG(INFO) << "checkpoint VisitExpr_ LetStmt";
  in_let = true;
  VARkey_to_arg = op->var.get();
  std::string value = PrintExpr(op->value);
  LOG(INFO) << value << "++++++++++++++++++++++++++++++++++";
  // Skip the argument retrieving assign statement
  auto it = var_idmap_.find(op->var.get());
  if (it == var_idmap_.end())  AllocVarID(op->var.get());
  //LOG(INFO) << "op->value type info" << op->value.type_info();
  //auto is_port = is_input.find(op->value.get());
  //if (is_port != is_input.end()) var_to_arg[op->value.get()] = op->var.get();
  /*if (op->var.type() != Handle() &&
      value.find("TVMArray") == std::string::npos &&
      value.find("arg") != 0) {
    PrintIndent();
    PrintType(op->var.type(), this->stream);
    this->stream << ' '
                 << vid
                 << " = " << value << ";\n";*
  }*/
  in_let = false;
  PrintStmt(op->body);
}

void CodeGenCHISEL::VisitExpr_(const Let* op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_ Let";
  in_let = true;
  VARkey_to_arg = op->var.get();
  CHECK(print_ssa_form_)
      << "LetExpr is only supported by print SSA form";
  std::string value = PrintExpr(op->value);

  CHECK(!var_idmap_.count(op->var.get()));
  var_idmap_[op->var.get()] = value;
  in_let = false;
  //LOG(INFO) << "op->value type info" << op->value.type_info();
  //auto is_port = is_input.find(op->value.get());
  //if (is_port != is_input.end()) var_to_arg[op->value.get()] = op->var.get();
}

inline void PrintBinaryIntrinsitc(const Call* op,
                                  const char *opstr,
                                  std::ostream& os,  // NOLINT(*)
                                  CodeGenCHISEL* p) {
  LOG(INFO) << "checkpoint PrintBinaryIntrinsitc";
  if (op->type.lanes() == 1) {
    CHECK_EQ(op->args.size(), 2U);
    os << '(';
    p->PrintExpr(op->args[0], os);
    os << opstr;
    p->PrintExpr(op->args[1], os);
    os << ')';
  } 
}

void CodeGenCHISEL::VisitStmt_(const Store* op) {
  stream_body << "\n";
  this->PrintIndent_body();
  LOG(INFO) << "!!!checkpoint VisitStmt_ Store";
  Type t = op->value.type();
  LOG(INFO) << t;
  
  // create a local wire and buffer  
  if (t.lanes() == 1) {
    std::string value = this->PrintExpr(op->value);
    LOG(INFO) << value;
    LOG(INFO) << "GOT VALUE VisitStmt_ Store ";
    std::string ref  = this->GetWire(t,op->buffer_var.get(), stream_body);
    LOG(INFO) << ref; 
    // update the unordered_map - is_input
    auto arg_find = var_to_arg.find(op->buffer_var.get());
    LOG(INFO) << GetVarID(arg_find->first);
    LOG(INFO) << GetVarID(arg_find->second);
    if ( arg_find != var_to_arg.end() ) {
      const Variable* arg = arg_find->second;
      is_input[arg] = false;
      LOG(INFO) << op->buffer_var << "false";
    }
    // print loop: traverse the unordered_map - var_to_arg
    for ( auto var_arg = var_to_arg.begin(); var_arg != var_to_arg.end(); ++ var_arg ){
      LOG(INFO) << "MAPPING var_to_arg: " << GetVarID(var_arg->first) << ", " << GetVarID(var_arg->second);
    }
    // print loop: traverse the unordered_map - is_input
    for ( auto port = is_input.begin(); port != is_input.end(); ++ port ){
      LOG(INFO) << GetVarID(port->first) << " : " << port->first << "; Bool:" <<port->second;
    }
    //wire_f_list[op->buffer_var.get()] = ref;
    /*
    this->PrintIndent_body();
    stream_body << "wire " << ref << " : ";
    this -> PrintType(t,stream_body);
    stream_body << "\n";
    this -> PrintIndent_body();
    stream_body << ref << " <= " << value << "\n";
    */
  } 
  else {
    CHECK(is_one(op->predicate))
        << "Predicated store is not supported";
    Expr base;
  }
  
  // hard coded for this example, updated later for general cases
  // Hardware IRs are required
  if (!in_for) {

  }
  LOG(INFO) << "!!!!!!!!!!!!!!!!!!!!!!end!!!!!!!!!!!!!!!!!!";
    
}

void CodeGenCHISEL::VisitStmt_(const For* op) {  
  //counter
  in_for = true;
  stream_body << "// data path \n";
  LOG(INFO) << "checkpoint VisitStmt_ For";
  // get the loop bound -> update later: find a way to pass to control logic
  //std::string extent = PrintExpr(op->extent);
  CHECK(is_zero(op->min));
  PrintIndent_body();
  stream_body << "when ((cnt < 5.U) && (io.a.valid)) {\n";
  PrintIndent_body();
  PrintIndent_body();
  stream_body << "b := b + 1.U\n";
  PrintIndent_body();
  PrintIndent_body();
  stream_body << "cnt := cnt + 1.U\n";
  PrintIndent_body();
  stream_body << "}.otherwise { \n";
  PrintIndent_body();
  PrintIndent_body();
  stream_body << "cnt := cnt + 1.U\n";
  PrintIndent_body();
  stream_body << "}\n";
  PrintIndent_body();
  stream_body << "io.b.bits := b\n";
  
  //how to get the names of the variables to be modified
  //maybe first go through the for loop body and save the arguments to be printed out
  this->PrintIndent_body();
   
  //PrintStmt(op->body); 
  //this->PrintIndent_body();
  stream_body << "// code ends here\n";
  // print loop: traverse the unordered_map - wire_f_list
  for ( auto wire_list = wire_f_list.begin(); wire_list != wire_f_list.end(); ++ wire_list ){
    LOG(INFO) << "===========wire_f_list:" << GetVarID(wire_list->first) << ";" << wire_list->second;
  }
  // print loop: traverse the unordered_map - vid_wire_reg
  for ( auto wire_reg = vid_wire_reg.begin(); wire_reg != vid_wire_reg.end(); ++ wire_reg ){
    LOG(INFO) << "===========vid_wire_reg:" << GetVarID(wire_reg->first) << ";" << wire_reg->second;
  }
  for ( auto x = wire_f_list.begin(); x != wire_f_list.end(); ++x ){
      std::string vid = GetVarID(x->first);
      PrintIndent_body();
      stream_body << vid << "_f <=  mux(done," << vid << "_r," << vid << "_" << wire_f_list[x->first] << ")\n";
  in_for = false;
  }
  
}

void CodeGenCHISEL::VisitStmt_(const Block *op) {
  LOG(INFO) << "checkpoint VisitStmt_ Block";
  LOG(INFO) << op->first;
  PrintStmt(op->first);
  if (op->rest.defined()) PrintStmt(op->rest);
}


std::string CodeGenCHISEL::GetStructRef(
    Type t, const Expr& buffer, const Expr& index, int kind) {
  LOG(INFO) << "checkpoint GetStructRef";
  if (kind < intrinsic::kArrKindBound_) {
    std::ostringstream os;
    os << "(((TVMArray*)";
    this->PrintExpr(buffer, os);
    os << ")";
    if (kind == intrinsic::kArrAddr) {
      os << " + ";
      this->PrintExpr(index, os);
      os << ")";
      return os.str();
    }
    os << '[';
    this->PrintExpr(index, os);
    os << "].";
    // other case: get fields.
    switch (kind) {
      case intrinsic::kArrData: os << "data"; break;
      case intrinsic::kArrShape: os << "shape"; break;
      case intrinsic::kArrStrides: os << "strides"; break;
      case intrinsic::kArrNDim: os << "ndim"; break;
      case intrinsic::kArrTypeCode: os << "dtype.code"; break;
      case intrinsic::kArrTypeBits: os << "dtype.bits"; break;
      case intrinsic::kArrTypeLanes: os << "dtype.lanes"; break;
      case intrinsic::kArrTypeFracs: os << "dtype.fracs"; break;
      case intrinsic::kArrDeviceId: os << "ctx.device_id"; break;
      case intrinsic::kArrDeviceType: os << "ctx.device_type"; break;
      default: LOG(FATAL) << "unknown field code";
    }
    os << ')';
    return os.str();
  } else {
    CHECK_LT(kind, intrinsic::kTVMValueKindBound_);
    std::ostringstream os;
    os << "(((TVMValue*)";
    this->PrintExpr(buffer, os);
    os << ")[" << index << "].";
    if (t.is_handle()) {
      os << "v_handle";
    } else if (t.is_float()) {
      os << "v_float64";
    } else if (t.is_int()) {
      os << "v_int64";
    } else {
      LOG(FATAL) << "donot know how to handle type" << t;
    }
    os << ")";
    return os.str();
  }
}

/*void CodeGenCHISEL::BindThreadIndex(const IterVar& iv) {
  LOG(INFO) << "checkpoint VisitExpr_ Var";
  LOG(FATAL) << "not implemented";
}*/

void CodeGenCHISEL::VisitStmt_(const AttrStmt* op) {
  LOG(INFO) << "checkpoint VisitStmt_ AttrStmt";
  this->PrintStmt(op->body);
}

void CodeGenCHISEL::VisitStmt_(const Allocate* op) {
  LOG(INFO) << "checkpoint VisitStmt_ Allocate";
  this->PrintStmt(op->body);
}

void CodeGenCHISEL::VisitStmt_(const IfThenElse* op) {
  LOG(INFO) << "checkpoint VisitStmt_ IfThenElse";
  std::string cond = PrintExpr(op->condition);
  // Skip the buffer data checking
  LOG(INFO) << "IfThenElse Condition"<< cond;
  if (std::regex_match(cond, std::regex("(not)\\(\\((arg)(.*)(== NULL)\\)")))
      LOG(INFO) << "it's a MATCH";
      return ;
  if (std::regex_match(cond, std::regex("!\\((tvm_handle_is_null)(.*)\\)")))
      return ;
  std::regex r("(not)\\(\\((arg)(.*)(== NULL)\\)");
  std::smatch m;
  regex_search(cond, m, r); 
  for (auto x : m) 
        LOG(INFO) << "condition match" << x << " "; 
  LOG(INFO) << "condition match?"; 
  PrintIndent();
  if (cond[0] == '(' && cond[cond.length() - 1] == ')') {
    stream << "if " << cond << " {\n";
  } else {
    stream << "if (" << cond << ") {\n";
  }
  int then_scope = BeginScope();
  PrintStmt(op->then_case);
  this->EndScope(then_scope);
  if (op->else_case.defined()) {
    PrintIndent();
    stream << "} else {\n";
    int else_scope = BeginScope();
    PrintStmt(op->else_case);
    this->EndScope(else_scope);
  }
  PrintIndent();
  stream << "}\n";
}

void CodeGenCHISEL::VisitExpr_(const Not *op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_ Not";
  LOG(INFO) << stream_body.str();
  os << "not(";
  PrintExpr(op->a, os);
  os <<')';
}

void CodeGenCHISEL::VisitStmt_(const AssertStmt* op) {
  LOG(INFO) << "checkpoint VisitStmt_ AssertStmt";
  LOG(INFO) << stream_body.str();
  this->PrintStmt(op->body);
}

void CodeGenCHISEL::VisitExpr_(const EQ *op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_ EQ";
  PrintBinaryExpr(op, "==", os, this);
}

void CodeGenCHISEL::VisitExpr_(const Cast *op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_ Cast";
  std::stringstream value;
  this->PrintExpr(op->value, value);
  //os << CastFromTo(value.str(), op->value.type(), op->type);
}

void CodeGenCHISEL::VisitExpr_(const Load* op, std::ostream& os) {  // NOLINT(*)
  LOG(INFO) << "checkpoint VisitExpr_ Load";
  // delcare type.
  if (op->type.lanes() == 1) {
    std::string ref = this->GetWire(op->type, op->buffer_var.get(), stream_body);
    os << ref;
  } else {
    CHECK(is_one(op->predicate))
        << "predicated load is not supported";
    /*Expr base;
    if (TryGetRamp1Base(op->index, op->type.lanes(), &base)) {
      std::string ref = GetVecLoad(op->type, op->buffer_var.get(), base);
      os << ref;
    } else {
      // The assignment below introduces side-effect, and the resulting value cannot
      // be reused across multiple expression, thus a new scope is needed
      int vec_scope = BeginScope();

      // load seperately.
      std::string svalue = GetUniqueName("_");
      this->PrintIndent();
      this->PrintType(op->type, stream);
      stream << ' ' << svalue << ";\n";
      std::string sindex = SSAGetID(PrintExpr(op->index), op->index.type());
      std::string vid = GetVarID(op->buffer_var.get());
      Type elem_type = op->type.element_of();
      for (int i = 0; i < lanes; ++i) {
        std::ostringstream value_temp;
        if (!HandleTypeMatch(op->buffer_var.get(), elem_type)) {
          value_temp << "((";
          if (op->buffer_var.get()->type.is_handle()) {
            auto it = alloc_storage_scope_.find(op->buffer_var.get());
            if (it != alloc_storage_scope_.end()) {
              PrintStorageScope(it->second, value_temp);
              value_temp << ' ';
            }
          }
          PrintType(elem_type, value_temp);
          value_temp << "*)" << vid << ')';
        } else {
          value_temp << vid;
        }
        value_temp << '[';
        PrintVecElemLoad(sindex, op->index.type(), i, value_temp);
        value_temp << ']';
        PrintVecElemStore(svalue, op->type, i, value_temp.str());
      }
      os << svalue;
      EndScope(vec_scope);
    }*/
  }
}


void CodeGenCHISEL::VisitStmt_(const Evaluate *op) {
  LOG(INFO) << "checkpoint Evaluate";
  if (is_const(op->value)) return;
  /*const Call* call = op->value.as<Call>();
  if (call) {
    if (call->is_intrinsic(intrinsic::tvm_storage_sync)) {
      this->PrintStorageSync(call); return;
    } else if (call->is_intrinsic(intrinsic::tvm_struct_set)) {
      CHECK_EQ(call->args.size(), 4);
      std::string value = PrintExpr(call->args[3]);
      std::string ref = GetStructRef(
          call->args[3].type(),
          call->args[0],
          call->args[1],
          call->args[2].as<IntImm>()->value);
      this->PrintIndent();
      this->stream << ref << " = " << value << ";\n";
      return;
    }
  }*/
  std::string vid = this->PrintExpr(op->value);
  this->PrintIndent_body();
  this->stream_body << "(void)" << vid << ";\n";
}

void CodeGenCHISEL::VisitExpr_(const Call *op, std::ostream& os) {  // NOLINT(*)
  if (op->call_type == Call::Extern ||
      op->call_type == Call::PureExtern) {
    os << op->name << "(";
    for (size_t i = 0; i < op->args.size(); i++) {
      this->PrintExpr(op->args[i], os);
      if (i < op->args.size() - 1) {
        os << ", ";
      }
    }
    os << ")";
  } else if (op->is_intrinsic(Call::bitwise_and)) {
    PrintBinaryIntrinsitc(op, " & ", os, this);
  } else if (op->is_intrinsic(Call::bitwise_xor)) {
    PrintBinaryIntrinsitc(op, " ^ ", os, this);
  } else if (op->is_intrinsic(Call::bitwise_or)) {
    PrintBinaryIntrinsitc(op, " | ", os, this);
  } else if (op->is_intrinsic(Call::bitwise_not)) {
    CHECK_EQ(op->args.size(), 1U);
    os << "(~";
    this->PrintExpr(op->args[0], os);
    os << ')';
  } else if (op->is_intrinsic(Call::shift_left)) {
    PrintBinaryIntrinsitc(op, " << ", os, this);
  } else if (op->is_intrinsic(Call::shift_right)) {
    PrintBinaryIntrinsitc(op, " >> ", os, this);
  } else if (op->is_intrinsic(intrinsic::tvm_if_then_else)) {
    os << "(";
    PrintExpr(op->args[0], os);
    os << " ? ";
    PrintExpr(op->args[1], os);
    os << " : ";
    PrintExpr(op->args[2], os);
    os << ")";
  } else if (op->is_intrinsic(intrinsic::tvm_address_of)) {
    const Load *l = op->args[0].as<Load>();
    CHECK(op->args.size() == 1 && l);
    os << "((";
    this->PrintType(l->type.element_of(), os);
    os << " *)" << this->GetVarID(l->buffer_var.get())
       << " + ";
    this->PrintExpr(l->index, os);
    os << ')';
  } else if (op->is_intrinsic(intrinsic::tvm_struct_get)) {
    CHECK_EQ(op->args.size(), 3U);
    os << GetStructRef(
        op->type, op->args[0], op->args[1],
        op->args[2].as<IntImm>()->value);
  } else if (op->is_intrinsic(intrinsic::tvm_handle_is_null)) {
    CHECK_EQ(op->args.size(), 1U);
    os << "(";
    this->PrintExpr(op->args[0], os);
    os << " == NULL)";
  } else {
    if (op->call_type == Call::Intrinsic ||
        op->call_type == Call::PureIntrinsic) {
      LOG(FATAL) << "Unresolved intrinsic " << op->name
                 << " with return type " << op->type;
    } else {
      LOG(FATAL) << "Unresolved call type " << op->call_type;
    }
  }
}

}  // namespace codegen
}  // namespace tvm
