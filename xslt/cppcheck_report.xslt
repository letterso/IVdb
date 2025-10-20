<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="html" indent="yes" encoding="UTF-8" omit-xml-declaration="yes"/>

<xsl:template match="/">
<html>
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Cppcheck 报告</title>
  <style>
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      margin: 20px;
      color: #333;
      background-color: #f8f9fa;
    }
    .header {
      background: linear-gradient(135deg, #1a237e, #0d47a1); /* 加深蓝色梯度 */
      color: white;
      text-align: center;
      padding: 25px 30px;
      border-radius: 8px;
      margin-bottom: 25px;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2); /* 加深阴影 */
    }
    .header h1 {
      font-weight: 700; /* 加粗字体 */
      margin-bottom: 8px;
      font-size: 2.4rem; /* 增大字号 */
      text-shadow: 0 2px 4px rgba(0,0,0,0.3); /* 添加文字阴影 */
      letter-spacing: 0.5px; /* 增加字间距 */
    }
    .header p {
      opacity: 0.9;
      font-size: 1.1rem;
      text-shadow: 0 1px 2px rgba(0,0,0,0.2); /* 副标题也添加阴影 */
    }
    @media (max-width: 768px) {
      .header h1 {
        font-size: 1.8rem;
      }
    }
    table {
      border-collapse: collapse;
      width: 100%;
      margin-top: 15px;
      box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
      background-color: white;
      border-radius: 8px;
      overflow: hidden;
    }
    th, td {
      border: 1px solid #f0f0f0;
      padding: 14px 20px;
      text-align: left;
      vertical-align: top;
    }
    th {
      background-color: #f5f7ff;
      color: #1a237e;
      font-weight: 600;
      border-bottom: 2px solid #e0e0e0;
    }
    tr:hover {
      background-color: #f8fbff;
    }
    .severity-tag {
      display: inline-block;
      padding: 5px 12px;
      border-radius: 14px;
      font-size: 0.85rem;
      font-weight: 600;
      text-align: center;
      min-width: 80px;
    }
    .error .severity-tag { 
      background-color: #ffcdd2; 
      color: #d32f2f; 
    }
    .warning .severity-tag { 
      background-color: #ffe0b2; 
      color: #f57c00; 
    }
    .style .severity-tag { 
      background-color: #bbdefb; 
      color: #1976d2; 
    }
    .information .severity-tag { 
      background-color: #e1bee7; 
      color: #7b1fa2; 
    }
    .location {
      font-family: monospace;
      font-size: 0.9em;
      color: #555;
    }
    .summary {
      background-color: white;
      padding: 25px;
      border-radius: 8px;
      box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
      margin: 20px 0;
    }
    .summary-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 18px;
      margin-top: 10px;
    }
    .summary-item {
      background: white;
      padding: 20px 15px;
      border-radius: 8px;
      box-shadow: 0 2px 6px rgba(0, 0, 0, 0.07);
      text-align: center;
      border: 1px solid #e0e0e0;
      transition: all 0.3s ease;
    }
    .summary-item:hover {
      transform: translateY(-3px);
      box-shadow: 0 5px 15px rgba(0, 0, 0, 0.1);
    }
    .stats-label {
      font-size: 0.95rem;
      color: #616161;
      margin-bottom: 8px;
      font-weight: 500;
    }
    .stats-value {
      font-weight: 700;
      font-size: 2.2rem;
    }
    .error-count { color: #d32f2f; }
    .warning-count { color: #f57c00; }
    .style-count { color: #1976d2; }
    .information-count { color: #7b1fa2; }
    .footer {
      text-align: center;
      margin-top: 30px;
      color: #757575;
      font-size: 0.9rem;
    }
  </style>
</head>
<body>
  <div class="header">
    <h1>Cppcheck 静态分析报告</h1>
    <p>版本 <xsl:value-of select="results/cppcheck/@version"/> | 报告格式版本 <xsl:value-of select="results/@version"/></p>
  </div>
  
  <!-- 汇总统计 -->
  <div class="summary">
    <h2>📊 问题概览</h2>
    <div class="summary-grid">
      <xsl:variable name="errors" select="count(results/errors/error[@severity='error'])"/>
      <xsl:variable name="warnings" select="count(results/errors/error[@severity='warning'])"/>
      <xsl:variable name="styles" select="count(results/errors/error[@severity='style'])"/>
      <xsl:variable name="infos" select="count(results/errors/error[@severity='information'])"/>
      <xsl:variable name="total" select="count(results/errors/error)"/>
      
      <div class="summary-item">
        <div class="stats-label">问题总数</div>
        <div class="stats-value"><xsl:value-of select="$total"/></div>
      </div>
      
      <div class="summary-item">
        <div class="stats-label">错误</div>
        <div class="stats-value error-count"><xsl:value-of select="$errors"/></div>
      </div>
      
      <div class="summary-item">
        <div class="stats-label">警告</div>
        <div class="stats-value warning-count"><xsl:value-of select="$warnings"/></div>
      </div>
      
      <div class="summary-item">
        <div class="stats-label">代码风格</div>
        <div class="stats-value style-count"><xsl:value-of select="$styles"/></div>
      </div>
      
      <div class="summary-item">
        <div class="stats-label">信息</div>
        <div class="stats-value information-count"><xsl:value-of select="$infos"/></div>
      </div>
    </div>
  </div>
  
  <!-- 问题详情表格 -->
  <h2>🔍 详细问题列表</h2>
  <table>
    <thead>
      <tr>
        <th>ID</th>
        <th>严重程度</th>
        <th>消息</th>
        <th>位置</th>
        <th>CWE</th>
      </tr>
    </thead>
    <tbody>
      <xsl:choose>
        <xsl:when test="count(results/errors/error) = 0">
          <tr>
            <td colspan="5" style="text-align:center; padding:30px; background-color:#f9f9f9;">
              🎉 恭喜！未发现任何静态分析问题
            </td>
          </tr>
        </xsl:when>
        <xsl:otherwise>
          <xsl:apply-templates select="results/errors/error"/>
        </xsl:otherwise>
      </xsl:choose>
    </tbody>
  </table>
  
  <div class="footer">
    <p>报告生成时间: <script>document.write(new Date().toLocaleString());</script></p>
  </div>
</body>
</html>
</xsl:template>

<xsl:template match="error">
  <tr class="{@severity}">
    <td><xsl:value-of select="@id"/></td>
    <td>
      <span class="severity-tag"><xsl:value-of select="@severity"/></span>
    </td>
    <td>
      <strong><xsl:value-of select="@msg"/></strong>
      <div style="margin-top:5px; font-size:0.9em; color:#666;">
        <xsl:value-of select="@verbose"/>
      </div>
    </td>
    <td class="location">
      <xsl:choose>
        <xsl:when test="location">
          <div><strong>文件:</strong> <xsl:value-of select="location/@file"/></div>
          <div><strong>行:</strong> <xsl:value-of select="location/@line"/></div>
          <xsl:if test="location/@column">
            <div><strong>列:</strong> <xsl:value-of select="location/@column"/></div>
          </xsl:if>
          <xsl:if test="location/symbol">
            <div><strong>符号:</strong> <xsl:value-of select="location/symbol"/></div>
          </xsl:if>
        </xsl:when>
        <xsl:otherwise>全局问题</xsl:otherwise>
      </xsl:choose>
    </td>
    <td>
      <xsl:choose>
        <xsl:when test="@cwe">
          <a href="https://cwe.mitre.org/data/definitions/{@cwe}.html" target="_blank" style="color:#1a237e; text-decoration:none;">
            CWE-<xsl:value-of select="@cwe"/>
          </a>
        </xsl:when>
        <xsl:otherwise>-</xsl:otherwise>
      </xsl:choose>
    </td>
  </tr>
</xsl:template>

</xsl:stylesheet>