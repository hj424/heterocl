/*!
 *  Copyright (c) 2018 by Contributors
 * \file build_vhls.cc
 * \brief Build FIRRTL modules for for loop from source.
 */
#include "./codegen_forfirrtl.h"
#include "../build_common.h"

namespace TVM {
namespace codegen {

std::string BuildFORFIRRTL(Array<LoweredFunc> funcs) {
  using TVM::runtime::Registry;

  CodeAnalysMerlinC ca;
  CodeGenFORFIRRTL cg;
  for (LoweredFunc f : funcs) {
    // 1st pass: Analyze AST and collect necessary information
    ca.AddFunction(f);
    str2tupleMap<std::string, Type> map_arg_type;
    map_arg_type = ca.Finish();
    // 2nd pass: Generate kernel code
    cg.AddFunction(f, map_arg_type);
  }
  std::string code = cg.Finish();

  if (const auto* f = Registry::Get("tvm_callback_forfirrtl_postproc")) {
    code = (*f)(code).operator std::string();
  }
  LOG(WARNING) << "FIRRTL doesn't have runtime, return kernel code";
  return code;
}

TVM_REGISTER_API("codegen.build_forfirrtl")
.set_body([](TVMArgs args, TVMRetValue* rv) {
    *rv = BuildFORFIRRTL(args[0]);
  });
}  // namespace codegen
}  // namespace tvm
