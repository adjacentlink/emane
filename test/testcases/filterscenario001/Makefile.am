harness_dir=$(top_srcdir)/test/harness/filterscenario
harness_cmd=filterscenario
harness_exe=$(harness_dir)/$(harness_cmd)
harness_bin_deps=$(harness_exe) $(top_srcdir)/src/libemane/.libs/libemane.so
harness_schema=filterscenario.xsd

testcase_inputs=     \
 filterscenario.xml

EXTRA_DIST=           \
 $(testcase_inputs)   \
 testcase-target.txt  \
 spectralmask.xml

all-local: testcase-output.txt
	@diff -q testcase-output.txt testcase-target.txt &> /dev/null

testcase-output.txt: $(testcase_inputs) $(harness_bin_deps)
	$(harness_exe) \
    --schema $(harness_dir)/$(harness_schema) \
    --mask spectralmask.xml \
    filterscenario.xml &> testcase-output.txt

clean-local:
	rm -f testcase-output.txt
