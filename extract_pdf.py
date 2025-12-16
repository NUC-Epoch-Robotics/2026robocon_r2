import PyPDF2

# 打开PDF文件
with open('rules.pdf', 'rb') as pdf_file:
    # 创建PDF阅读器对象
    pdf_reader = PyPDF2.PdfReader(pdf_file)
    
    # 初始化文本变量
    text = ''
    
    # 遍历所有页面并提取文本
    for page_num in range(len(pdf_reader.pages)):
        page = pdf_reader.pages[page_num]
        text += page.extract_text() + '\n'
    
    # 打印提取的文本（前50000字符）
    print(text[:50000])