/*!
 *  Copyright (c) 2016 by Contributors
 * \file codegen_chisel.h
 * \brief: Common utilities to generated CHISEL code.
 */
#ifndef TVM_CODEGEN_CODEGEN_CHISEL_H_
#define TVM_CODEGEN_CODEGEN_CHISEL_H_

#include <tvm/codegen.h>
#include <tvm/packed_func_ext.h>
#include <string>
#include "../codegen_c.h"
#include "../merlinc/codeanalys_merlinc.h"

namespace TVM {
namespace codegen {

using namespace ir;
/*!
 * \brief A base class to generate C code.
 *
 *  CodeGenC have two modes: generate SSA formed C code or normal form.
 *
 * **NOTE** CodeGenC does not aim at generating C codes consumed by MSVC or GCC,
 * Rather, it's providing infrastructural abstraction for C variants like CUDA
 * and OpenCL-C. You might find some odd variant features, e.g., type `int3` for
 * a vector of 3 `int`s. For native C code generator, see `CodeGenLLVM`.
 */
class CodeGenCHISEL final : public CodeGenC { /*:
      public ExprFunctor<void(const Expr&, std::ostream&)>,
      public StmtFunctor<void(const Stmt&)>,
      public CodeGenSourceBase {*/
 public:
  /*!
   * \brief Initialize the code generator.
   * \param output_ssa Whether output SSA.
   */
  void Init(bool output_ssa);
  /*!
   * \brief Add the function to the generated module.
   * \param f The function to be compiled.
   */
  void AddFunction(LoweredFunc f, str2tupleMap<std::string, Type> map_arg_type);
  /*!
   * \brief Finalize the compilation and return the code.
   * \return The code.
   */
  std::string Finish();

  /*! \brief print the current indented value */
  void PrintIndent_body();
  int BeginScope_body();
  void EndScope_body(int scope_id);

  /*!
   * \brief Print the expression n(or its ssa id if in ssa mode) into os
   * \param n The expression to be printed.
   * \param os The output stream
   */
  void PrintExpr(const Expr& n, std::ostream& os);
  /*!
   * \brief Same as PrintExpr, but simply returns result string
   * \param n The expression to be printed.
   */
  std::string PrintExpr(const Expr& n) {
    std::ostringstream os;
    PrintExpr(n, os);
    return os.str();
  }

  /*!
   * Print Type represetnation of type t.
   * \param t The type representation.
   * \param os The stream to print the ctype into
   */
  virtual void PrintType(Type t, std::ostream& os); // NOLINT(*)
     /*!
   * \brief Same as PrintStmt, but simply returns result string
   * \param n The expression to be printed.
   */
  void PrintStmt(const Stmt& n) {
    VisitStmt(n);
  }
  // The following parts are overloadable print operations.
  /*!
   * \brief Initialize codegen state for generating f.
   * \param f The function to be compiled.
   */
  virtual void InitFuncState(LoweredFunc f);
  // expression
  //for, block, store, if, add, getbit
  //for, block
  void VisitExpr_(const Variable* op, std::ostream& os) override;  // NOLINT(*)
  void VisitExpr_(const Add* op, std::ostream& os) override;  // NOLINT(*)
  
  void VisitExpr_(const UIntImm* op, std::ostream& os) override;  // NOLINT(*)
  void VisitExpr_(const IntImm* op, std::ostream& os) override;  // NOLINT(*)
  // statment
  void VisitStmt_(const LetStmt* op) override;
  void VisitStmt_(const Store* op) override;
  void VisitStmt_(const For* op) override;
  void VisitStmt_(const Block* op) override;

  void VisitStmt_(const AttrStmt* op) override;
  void VisitStmt_(const Allocate* op) override;
  void VisitStmt_(const IfThenElse* op) override;
  void VisitExpr_(const Not* op, std::ostream& os) override;  // NOLINT(*)
  void VisitStmt_(const AssertStmt* op) override;
  void VisitExpr_(const EQ* op, std::ostream& os) override;
  void VisitExpr_(const Cast* op, std::ostream& os) override;
  void VisitExpr_(const Load* op, std::ostream& os) override;  // NOLINT(*)
  void VisitStmt_(const Evaluate* op) override;
  void VisitExpr_(const Call* op, std::ostream& os) override;  // NOLINT(*)
  void VisitExpr_(const Let* op, std::ostream& os) override;  // NOLINT(*)


  std::string GetWire(Type t, const Variable* v, std::ostream& os);

  //virtual void BindThreadIndex(const IterVar& iv); // NOLINT(*)


 protected:
  // Print reference to struct location
  std::string GetStructRef(
      Type t, const Expr& buffer, const Expr& index, int kind);
    /*!
   * \brief If buffer is allocated as type t.
   * \param buf_var The buffer variable.
   * \param t The type to be checked.
   */
  bool HandleTypeMatch(const Variable* buf_var, Type t) const;
  /*!
   * \brief Register the data type of buf_var
   * \param buf_var The buffer variable.
   * \param t The type to be checked.
   */
  void RegisterHandleType(const Variable* buf_var, Type t);
  // override
  /*void PrintSSAAssign(
      const std::string& target, const std::string& src, Type t) final;*/

  /*! \brief restrict keyword */
  std::string restrict_keyword_{""};
  /*! \brief the storage scope of allocation */
  std::unordered_map<const Variable*, std::string> alloc_storage_scope_;
  /*! \brief the data type of allocated buffers */
  std::unordered_map<const Variable*, Type> handle_data_type_;

 private:
  //implement datatype for storing out and i
  std::unordered_map<const Variable*, std::string> wire_f_list;
 // std::unordered_map<std::string, std::string> reg_def;
  std::unordered_map<const Variable*, std::string> vid_wire_reg;
  std::unordered_map<const Variable*, bool> is_input;
  std::ostringstream stream_body;
  /*! \brief The current indentation value of stream_print */
  int indent_body{0};
  std::vector<bool> scope_mark_body;
  bool in_for{false};
  /*! \brief whether to print in SSA form */
  bool print_ssa_form_{false};
  /*! \brief set of volatile buf access */
  std::unordered_set<const Variable*> volatile_buf_;
  std::unordered_map<const Variable*, int> buf_length_map_;
  std::unordered_map<const Variable*, const Variable*> var_to_arg;
  bool in_let{false};
  const Variable* VARkey_to_arg;
};

}  // namespace codegen
}  // namespace tvm
#endif  // TVM_CODEGEN_CODEGEN_CHISEL_H_
