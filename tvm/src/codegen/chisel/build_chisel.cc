/*!
 *  Copyright (c) 2018 by Contributors
 * \file build_vhls.cc
 * \brief: Build CHISEL modules from source.
 */
#include "./codegen_chisel.h"
#include "../build_common.h"

namespace TVM {
namespace codegen {

std::string BuildCHISEL(Array<LoweredFunc> funcs) {
  using TVM::runtime::Registry;

  CodeAnalysMerlinC ca;
  CodeGenCHISEL cg;
  for (LoweredFunc f : funcs) {
    // 1st pass: Analyze AST and collect necessary information
    ca.AddFunction(f);
    str2tupleMap<std::string, Type> map_arg_type;
    map_arg_type = ca.Finish();
    // 2nd pass: Generate kernel code
    cg.AddFunction(f, map_arg_type);
  }
  std::string code = cg.Finish();

  if (const auto* f = Registry::Get("tvm_callback_chisel_postproc")) {
    code = (*f)(code).operator std::string();
  }
  LOG(WARNING) << "CHISEL doesn't have runtime, return kernel code";
  return code;
}

TVM_REGISTER_API("codegen.build_chisel")
.set_body([](TVMArgs args, TVMRetValue* rv) {
    *rv = BuildCHISEL(args[0]);
  });
}  // namespace codegen
}  // namespace tvm
