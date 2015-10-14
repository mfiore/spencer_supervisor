
(cl:in-package :asdf)

(defsystem "appl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "appl_request" :depends-on ("_package_appl_request"))
    (:file "_package_appl_request" :depends-on ("_package"))
  ))